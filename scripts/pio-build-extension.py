#!/usr/bin/env python3
import os
from os.path import join

Import("env")

print("Current CLI targets", COMMAND_LINE_TARGETS)
print("Current Build targets", BUILD_TARGETS)

def post_program_action(source, target, env):
    print("Program has been built!")
    program_path = target[0].get_abspath()
    print("Program path", program_path)
    # Use case: sign a firmware, do any manipulations with ELF, etc
    # env.Execute(f"sign --elf {program_path}")

# env.AddPostAction("$PROGPATH", post_program_action)

print("PROGPATH is $PROGPATH")

def tst(source, target, env):
   print("tst")

# We generate all parts, using post actions or dependencies. Now merge all to a firmware bin file....
def generate_image(source, target, env):
  env.Execute('"$PYTHONEXE" -m esptool --chip esp32 merge-bin -o $BUILD_DIR/firmware-image.bin --flash-mode dio --flash-size 4MB 0x1000 $BUILD_DIR/bootloader.bin 0x8000 $BUILD_DIR/partitions.bin 0x10000 $BUILD_DIR/firmware.bin 0x310000 $BUILD_DIR/fonts.bin 0x320000 $BUILD_DIR/littlefs.bin --target-offset 0x1000')


# default target is elf file (if not target buildfs or uploadfs on command line)
# so this target will build elf file and file system bin as dependency
# Also, as a post actino for the elf file, we generate fonts.bin
#target_fsbin = env.DataToBin(join("$BUILD_DIR", "${ESP32_FS_IMAGE_NAME}"), "$PROJECT_DATA_DIR")
#env.AddCustomTarget(
#  "firmware",
#  ["$BUILD_DIR/${PROGNAME}.bin", target_fsbin],
#  generate_image)

def build_web(source, target, env):
    web_dir = env.subst("$PROJECT_DIR/web")
    print("Building web UI into RX_FSK/data")
    status = env.Execute('npm --prefix "%s" run build' % web_dir)
    if status:
        return status
    with open(target[0].get_abspath(), "w") as stamp:
        stamp.write("ok\n")
    return 0

# npm writes RX_FSK/data before the filesystem image is packed.
web_stamp = env.Command(join("$BUILD_DIR", "web-build.stamp"), [], build_web)
env.AlwaysBuild(web_stamp)

target_fsbin = env.DataToBin(
    join("$BUILD_DIR", "${ESP32_FS_IMAGE_NAME}"),
    "$PROJECT_DATA_DIR",
)
env.Depends(target_fsbin, web_stamp)
env.AlwaysBuild(target_fsbin)

env.AddCustomTarget(
  "firmware",
  ["$BUILD_DIR/${PROGNAME}.bin", target_fsbin],
  generate_image
)

# After generating the elf file, generate fonts.bin as well.
# The Arduino core rebuild does not compile project sources, so the old
# object directory is not created for us. g++ will not create it either.
def build_fonts_bin(source, target, env):
    out_dir = env.subst(join("$BUILD_DIR", "src", "src", "fonts"))
    os.makedirs(out_dir, exist_ok=True)
    obj = join(out_dir, "fonts.cpp.o")
    fonts_bin = env.subst(join("$BUILD_DIR", "fonts.bin"))
    print("Building %s" % fonts_bin)
    status = env.Execute(" ".join([
        "xtensa-esp32-elf-g++", "-fno-lto", "-c",
        "RX_FSK/src/fonts/fonts.cpp", "-o", obj,
    ]))
    if status:
        return status
    return env.Execute(" ".join([
        "xtensa-esp32-elf-ld", "-T", "fontlink.ld", "--oformat=binary",
        "-o", fonts_bin, obj,
    ]))

env.AddPostAction("$PROGPATH", build_fonts_bin)



env.AddCustomTarget(
  "uploadfonts",
  "$BUILD_DIR/${PROGNAME}.bin",
  "scripts/uploadfonts.py $BUILD_DIR/fonts.bin $PARTITIONS_TABLE_CSV"
)
