import subprocess
from pathlib import Path
from SCons.Script import COMMAND_LINE_TARGETS, Alias

Import("env")

print("[force-buildfs] loaded, COMMAND_LINE_TARGETS =", COMMAND_LINE_TARGETS)

def make_good_littlefs(*args, **kwargs):
    proj = Path(env["PROJECT_DIR"])
    data_dir = proj / "RX_FSK" / "data"
    out_bin  = Path(env.subst("$BUILD_DIR")) / "littlefs.bin"

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

    # sanity check
    print("[force-buildfs] sanity list:")
    subprocess.check_call([str(tool), "-b", "4096", "-p", "256", "-l", str(out_bin)])

def try_add_post(node_or_name):
    try:
        env.AddPostAction(node_or_name, make_good_littlefs)
        print("[force-buildfs] POST hooked to", node_or_name)
    except Exception as ex:
        print("[force-buildfs] POST hook failed for", node_or_name, ":", ex)

def try_add_pre(name):
    try:
        env.AddPreAction(name, make_good_littlefs)
        print("[force-buildfs] PRE hooked to", name)
    except Exception as ex:
        print("[force-buildfs] PRE hook failed for", name, ":", ex)

# --- buildfs: overwrite router output AFTER it runs ---
if "buildfs" in COMMAND_LINE_TARGETS:
    # Hook both the produced file and the alias, because the router is non-standard
    try_add_post(env.File(env.subst("$BUILD_DIR/littlefs.bin")))
    try_add_post(Alias("buildfs"))

# --- uploadfs/upload: ensure correct image right BEFORE flashing ---
if "uploadfs" in COMMAND_LINE_TARGETS:
    try_add_pre("uploadfs")

if "upload" in COMMAND_LINE_TARGETS:
    try_add_pre("upload")

env["MAKE_GOOD_LITTLEFS"] = make_good_littlefs
