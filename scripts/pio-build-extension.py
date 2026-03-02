#!/usr/bin/env python3
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
  env.Execute("esptool.py --chip esp32 merge_bin -o $BUILD_DIR/firmware-image.bin --flash_mode dio --flash_size 4MB 0x1000 $BUILD_DIR/bootloader.bin 0x8000 $BUILD_DIR/partitions.bin 0x10000 $BUILD_DIR/firmware.bin 0x310000 $BUILD_DIR/fonts.bin 0x320000 $BUILD_DIR/littlefs.bin --target-offset 0x1000")


# default target is elf file (if not target buildfs or uploadfs on command line)
# so this target will build elf file and file system bin as dependency
# Also, as a post actino for the elf file, we generate fonts.bin
#target_fsbin = env.DataToBin(join("$BUILD_DIR", "${ESP32_FS_IMAGE_NAME}"), "$PROJECT_DATA_DIR")
#env.AddCustomTarget(
#  "firmware",
#  ["$BUILD_DIR/${PROGNAME}.bin", target_fsbin],
#  generate_image)

env.AddCustomTarget(
  "firmware",
  ["$BUILD_DIR/${PROGNAME}.bin"],
  generate_image
)

# After generating the elf file, generate fonts.bin as well
env.AddPostAction("$PROGPATH", 
  env.VerboseAction(" ".join([
    "xtensa-esp32-elf-g++", "-fno-lto", "-c", "RX_FSK/src/fonts/fonts.cpp", "-o", "$BUILD_DIR/src/src/fonts/fonts.cpp.o" ]),
    "Building $BUILD_DIR/fonts.bin"))
env.AddPostAction("$PROGPATH", 
  env.VerboseAction(" ".join([
    "xtensa-esp32-elf-ld", "-T", "fontlink.ld", "--oformat=binary", "-o", "$BUILD_DIR/fonts.bin", "$BUILD_DIR/src/src/fonts/fonts.cpp.o" ]),
    "Building $BUILD_DIR/fonts.bin"))



env.AddCustomTarget(
  "uploadfonts",
  "$BUILD_DIR/${PROGNAME}.bin",
  "scripts/uploadfonts.py $BUILD_DIR/fonts.bin $PARTITIONS_TABLE_CSV"
)
