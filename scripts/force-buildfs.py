import subprocess
from pathlib import Path
from SCons.Script import Alias, COMMAND_LINE_TARGETS

Import("env")

print("[force-buildfs] loaded, COMMAND_LINE_TARGETS =", COMMAND_LINE_TARGETS)

def force_buildfs(target, source, env):
    proj = Path(env["PROJECT_DIR"])
    data_dir = proj / "RX_FSK" / "data"

    build_dir = Path(env.subst("$BUILD_DIR"))
    out_bin = build_dir / "littlefs.bin"

    tool = Path.home() / ".platformio" / "packages" / "tool-mklittlefs" / "mklittlefs"
    if not tool.exists():
        tool = Path.home() / ".platformio" / "tools" / "tool-mklittlefs" / "mklittlefs"

    cmd = [
        str(tool),
        "-c", str(data_dir),
        "-s", "0xD0000",
        "-b", "4096",
        "-p", "256",
        str(out_bin),
    ]

    print("[force-buildfs] generating:", " ".join(cmd))
    out_bin.parent.mkdir(parents=True, exist_ok=True)
    subprocess.check_call(cmd)

    print("[force-buildfs] sanity list:")
    subprocess.check_call([str(tool), "-b", "4096", "-p", "256", "-l", str(out_bin)])


if "buildfs" in COMMAND_LINE_TARGETS:
    # Hook the alias (target) AND the output file
    env.AddPostAction("buildfs", force_buildfs)
    env.AddPostAction(env.File(env.subst("$BUILD_DIR/littlefs.bin")), force_buildfs)
    print("[force-buildfs] hooked post-action to alias 'buildfs' and littlefs.bin")
