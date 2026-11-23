#!/usr/bin/env python3
"""Real vs simulated arm as a video (issue #2): the arm posed with the REAL logged joint positions (solid), the MuJoCo
servo model's replay of the same goals as a translucent orange GHOST on top, and a strip chart (goal / real / sim) with
a moving cursor underneath. Where the ghost covers the arm exactly, they agree.

    MUJOCO_GL=glfw python3 tools/make_replay_video.py sysid_logs/<stamp>_<payload> --joint 3 \
        [--fit-on <other session>] [--t0 0 --t1 61] [--speed 1] [--out docs/videos/m3.mp4]

--fit-on: take kp, dead zone and static friction from ANOTHER run of the same joint (held-out check); without it they
are fitted on the replayed run itself. Only the tested joint moves in a sysid run; the others are shown at the base pose.
Needs mujoco (glfw or egl rendering), matplotlib, Pillow, imageio + imageio-ffmpeg.
"""
import argparse
import io
import os
import sys

import imageio.v2 as imageio
import matplotlib
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from elrobot_model import Arm, read_meta_base  # noqa: E402
from servo_sim import ServoParams, fit_kp, load_csv, replay  # noqa: E402

W, H, CHART_H = 1280, 720, 230
GHOST = np.array([255, 140, 30], float)
ALPHA = 0.42


def fit_params(session, joint, log):
    """kp from the log, then dead zone and static friction by the replay grid (as servo_sim.py replay --fit)."""
    kp = fit_kp(log)
    best = None
    for dz in (4.0, 8.0, 12.0):
        for fr in (0.02, 0.05, 0.15, 0.3, 0.5):
            p = ServoParams(kp_duty=kp, dead_zone=dz, static_friction=fr)
            pos, _ = replay(session, joint, p, log)
            rms = float(np.sqrt(np.mean((pos - log["position"]) ** 2)))
            if best is None or rms < best[0]:
                best = (rms, p)
    return best[1]


def chart(t, goal, real, sim, t0, t1, title):
    fig, ax = plt.subplots(figsize=(W / 100, CHART_H / 100), dpi=100)
    m = (t >= t0) & (t <= t1)
    ax.plot(t[m], goal[m], color="#888", lw=1, label="goal")
    ax.plot(t[m], real[m], color="#1f77b4", lw=1.6, label="real arm")
    ax.plot(t[m], sim[m], color="#ff8c1e", lw=1.4, ls="--", label="MuJoCo")
    ax.set_xlim(t0, t1)
    ax.set_ylabel("ticks")
    ax.legend(loc="upper right", fontsize=8, ncol=3)
    ax.set_title(title, fontsize=9)
    fig.tight_layout(pad=0.3)
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100)
    plt.close(fig)
    img = Image.open(buf).convert("RGB").resize((W, CHART_H))
    # x pixel of a time: from the axes position
    bb = ax.get_position()
    return img, (bb.x0 * W, bb.x1 * W, t0, t1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--joint", type=int, required=True)
    ap.add_argument("--fit-on", help="another session of the same joint to take the parameters from (held out)")
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--azimuth", type=float, default=150)
    ap.add_argument("--elevation", type=float, default=-15)
    ap.add_argument("--distance", type=float, default=0.42)
    ap.add_argument("--look", default="Gripper_Base_v1_1", help="body to look at")
    ap.add_argument("--look-dz", type=float, default=0.0, help="shift the look-at point up (m)")
    ap.add_argument("--magnify", type=float, default=1.0,
                    help="scale both arms' excursion from the base pose by this factor (labelled; the chart stays true)")
    ap.add_argument("--out")
    a = ap.parse_args()

    j = a.joint - 1
    log = load_csv(a.session, a.joint)
    src = a.fit_on or a.session
    p = fit_params(src, a.joint, load_csv(src, a.joint) if a.fit_on else log)
    sim_pos, _ = replay(a.session, a.joint, p, log)
    t, goal, real = log["t"], log["goal"], log["position"]
    t1 = a.t1 if a.t1 is not None else float(t[-1])
    m = (t >= a.t0) & (t <= t1)
    rms = float(np.sqrt(np.mean((sim_pos[m] - real[m]) ** 2)))
    held = "HELD OUT: fitted on another run" if a.fit_on else "fitted on this run"
    print("M%d: kp %.4f, dead zone %.0f, friction %.2f (%s); RMS %.2f ticks (%.2f deg) over %.0f-%.0f s" % (
        a.joint, p.kp_duty, p.dead_zone, p.static_friction, held, rms, rms * 360 / 4096, a.t0, t1))

    base = read_meta_base(a.session)
    arm = Arm()
    mdl, dat = arm.m, arm.d
    mdl.vis.global_.offwidth, mdl.vis.global_.offheight = W, H
    mdl.vis.headlight.ambient[:] = (0.45, 0.45, 0.45)
    mdl.vis.headlight.diffuse[:] = (0.65, 0.65, 0.65)
    rgba0 = mdl.geom_rgba.copy()
    grey = np.tile([0.42, 0.55, 0.72, 1.0], (mdl.ngeom, 1))  # the real arm: steel blue
    ren = mujoco.Renderer(mdl, H - CHART_H, W)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    arm.set_ticks(base)
    cam.lookat[:] = dat.xpos[mdl.body(a.look).id] + np.array([0.0, 0.0, a.look_dz])
    cam.azimuth, cam.elevation, cam.distance = a.azimuth, a.elevation, a.distance

    def add_table(scene):
        if scene.ngeom >= scene.maxgeom:
            return
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, np.array([0.6, 0.6, 0.002]), np.array([0.25, 0.0, -0.002]),
                            np.eye(3).flatten(), np.array([0.86, 0.78, 0.64, 1.0], np.float32))
        scene.ngeom += 1

    def render(ticks, rgba, seg=False):
        mdl.geom_rgba[:] = rgba
        arm.set_ticks(ticks)
        if seg:
            ren.enable_segmentation_rendering()
            ren.update_scene(dat, cam)
            out = ren.render()
            ren.disable_segmentation_rendering()
            return out
        ren.update_scene(dat, cam)
        add_table(ren.scene)
        return ren.render()

    title = "M%d  real arm (blue) vs MuJoCo servo model (orange ghost)   RMS %.1f ticks = %.2f°   [%s]" % (
        a.joint, rms, rms * 360 / 4096, held)
    note = ("motion magnified x%g (both arms, same factor; chart in true ticks)" % a.magnify) if a.magnify != 1 else ""
    chart_img, (x0, x1, c0, c1) = chart(t, goal, real, sim_pos, a.t0, t1, "M%d position" % a.joint)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 18)
    except OSError:
        font = ImageFont.load_default()

    out = a.out or os.path.join(os.path.dirname(__file__), "..", "docs", "videos",
                                "M%d_%s%s.mp4" % (a.joint, os.path.basename(a.session.rstrip("/")), "_heldout" if a.fit_on else ""))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    writer = imageio.get_writer(out, fps=a.fps, codec="libx264", quality=8, macro_block_size=8)
    n = int((t1 - a.t0) / a.speed * a.fps)
    hh = H - CHART_H
    grad = np.linspace(0, 1, hh)[:, None, None]
    sky = (np.array([235, 241, 248], float) * (1 - grad) + np.array([200, 212, 228], float) * grad) * np.ones((1, W, 1))
    for k in range(n):
        tt = a.t0 + k * a.speed / a.fps
        rt = list(base)
        st = list(base)
        rt[j] = float(np.interp(tt, t, real))
        st[j] = float(np.interp(tt, t, sim_pos))
        rm, sm = list(base), list(base)
        rm[j] = base[j] + a.magnify * (rt[j] - base[j])
        sm[j] = base[j] + a.magnify * (st[j] - base[j])
        img = render(rm, grey).astype(float)
        bg = img.sum(axis=2) == 0  # nothing drawn: sky
        img[bg] = sky[bg]
        ghost = render(sm, np.tile([1.0, 0.55, 0.12, 1.0], (mdl.ngeom, 1))).astype(float)
        mask = render(sm, grey, seg=True)[:, :, 0] >= 0
        img[mask] = (1 - ALPHA) * img[mask] + ALPHA * ghost[mask]
        frame = Image.new("RGB", (W, H), (255, 255, 255))
        frame.paste(Image.fromarray(img.astype(np.uint8)), (0, 0))
        frame.paste(chart_img, (0, H - CHART_H))
        d = ImageDraw.Draw(frame)
        x = x0 + (tt - c0) / (c1 - c0) * (x1 - x0)
        d.line([(x, H - CHART_H + 8), (x, H - 28)], fill=(200, 0, 0), width=2)
        d.rectangle([(0, 0), (W, 30)], fill=(255, 255, 255))
        d.text((10, 6), title, fill=(20, 20, 20), font=font)
        d.rectangle([(0, H - CHART_H - 32), (W, H - CHART_H)], fill=(255, 255, 255))
        d.text((10, H - CHART_H - 28), "t = %5.1f s   real %4.0f   sim %4.0f   goal %4.0f ticks   %s" % (
            tt, rt[j], st[j], np.interp(tt, t, goal), note), fill=(20, 20, 20), font=font)
        writer.append_data(np.asarray(frame))
    writer.close()
    mdl.geom_rgba[:] = rgba0
    print("video %s (%d frames, %.0f s)" % (os.path.abspath(out), n, n / a.fps))


if __name__ == "__main__":
    main()
