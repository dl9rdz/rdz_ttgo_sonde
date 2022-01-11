#!/usr/bin/env python3
Import("env")

env.AddPostAction(
  "$BUILD_DIR/${PROGNAME}.elf",
  env.VerboseAction(" ".join([
    "xtensa-esp32-elf-ld", "-T", "fontlink.ld", "--oformat=binary", "-o", "$BUILD_DIR/fonts.bin", "$BUILD_DIR/src/src/fonts/fonts.cpp.o" ]),
    "Building $BUILD_DIR/fonts.bin")
)

env.AddCustomTarget(
  "uploadfonts",
  "$BUILD_DIR/fonts.bin",
  "scripts/uploadfonts.py $BUILD_DIR/fonts.bin $PARTITIONS_TABLE_CSV"
)
