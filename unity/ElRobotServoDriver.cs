// Drives the ElRobot's joints in MuJoCo (Unity plugin) with the St3215Servo model: motor torque into each joint's
// actuator every physics step, gearbox stiction as the joint's frictionloss. Same hook-up as RoboSim's
// BipedServoDriver (MjScene.postInitEvent / preUpdateEvent, MjHingeJoint / MjActuator lookup by name).
//
// Scene requirements (see docs/SERVO_MODEL.md, "Unity / C#"):
// - one MjActuator per joint, a plain torque motor: gear 1, no gain/bias of its own, ctrlrange >= the servo's peak
//   torque (or unlimited); its name = the joint's name;
// - no MuJoCo position servo on those joints: the position loop is in St3215Servo;
// - physics timestep 1 ms (the electrical and protection model are explicit, 1 ms keeps them accurate).
//
// Compile inside the Unity project (needs the MuJoCo Unity package and unsafe code enabled).

using System;
using Mujoco;
using UnityEngine;

namespace ElRobotSysId
{
    public sealed unsafe class ElRobotServoDriver : IDisposable
    {
        public readonly St3215Servo[] Servos;
        readonly string[] _names;
        readonly double[] _staticFriction;
        readonly GameObject _robot;
        MjScene _scene;
        readonly int[] _qpos, _dof, _actuator;
        bool _ready;

        /// <param name="names">Joint (and actuator) names, e.g. rev_motor_01 .. rev_motor_07.</param>
        /// <param name="servos">One model per joint, in the same order (parameters per joint: friction differs).</param>
        /// <param name="staticFrictionNm">Gearbox static friction per joint (N m), given to MuJoCo as frictionloss.</param>
        public ElRobotServoDriver(MjScene scene, GameObject robot, string[] names, St3215Servo[] servos, double[] staticFrictionNm)
        {
            if (names.Length != servos.Length || names.Length != staticFrictionNm.Length)
                throw new ArgumentException("names, servos and friction need one entry per joint");
            _scene = scene;
            _robot = robot;
            _names = names;
            Servos = servos;
            _staticFriction = staticFrictionNm;
            _qpos = new int[names.Length];
            _dof = new int[names.Length];
            _actuator = new int[names.Length];
            _scene.postInitEvent += OnPostInit;
            _scene.preUpdateEvent += OnPreUpdate;
        }

        public bool IsReady => _ready;

        /// <summary>Send a Goal Position (ticks) to joint i, like a bus write; it acts after the servo's delay.</summary>
        public void Command(int joint, double goalTicks) => Servos[joint].Command(goalTicks);

        public void Dispose()
        {
            if (_scene == null)
                return;
            _scene.postInitEvent -= OnPostInit;
            _scene.preUpdateEvent -= OnPreUpdate;
            _scene = null;
            _ready = false;
        }

        void OnPostInit(object sender, MjStepArgs e)
        {
            _ready = Resolve(e);
            if (!_ready)
                return;
            var m = e.model;
            for (int i = 0; i < _names.Length; i++)
            {
                int d = _dof[i];
                // Gearbox stiction. MuJoCo's frictionloss is soft by default: a sustained load creeps through it (a held
                // arm sagged 20+ ticks/s). A stiff constraint plus noslip iterations makes it hold like the real gearbox.
                m->dof_frictionloss[d] = _staticFriction[i];
                m->dof_solref[2 * d] = 0.002;
                m->dof_solref[2 * d + 1] = 1.0;
                m->dof_solimp[5 * d] = 0.99;
                m->dof_solimp[5 * d + 1] = 0.999;
                m->dof_solimp[5 * d + 2] = 0.001;
                m->dof_solimp[5 * d + 3] = 0.5;
                m->dof_solimp[5 * d + 4] = 2.0;
            }
            m->opt.noslip_iterations = Math.Max(m->opt.noslip_iterations, 10);
        }

        void OnPreUpdate(object sender, MjStepArgs e)
        {
            if (!_ready)
                return;
            double dt = e.model->opt.timestep;
            var data = e.data;
            for (int i = 0; i < _names.Length; i++)
                data->ctrl[_actuator[i]] = Servos[i].Step(dt, data->qpos[_qpos[i]], data->qvel[_dof[i]]);
        }

        bool Resolve(MjStepArgs e)
        {
            var joints = _robot.GetComponentsInChildren<MjHingeJoint>(true);
            var actuators = _robot.GetComponentsInChildren<MjActuator>(true);
            for (int i = 0; i < _names.Length; i++)
            {
                _qpos[i] = _dof[i] = _actuator[i] = -1;
                foreach (var j in joints)
                    if (j.name == _names[i])
                    {
                        _qpos[i] = j.QposAddress;
                        _dof[i] = j.DofAddress;
                    }
                foreach (var a in actuators)
                    if (a.name == _names[i])
                        _actuator[i] = a.MujocoId;
                if (_qpos[i] < 0 || _actuator[i] < 0)
                {
                    Debug.LogError($"[ElRobotServoDriver] joint '{_names[i]}' has no hinge or no motor under the robot root.");
                    return false;
                }
            }
            if (Math.Abs(e.model->opt.timestep - 0.001) > 1e-9)
                Debug.LogWarning($"[ElRobotServoDriver] timestep {e.model->opt.timestep} s: the servo model was validated at 0.001 s.");
            return true;
        }
    }
}
