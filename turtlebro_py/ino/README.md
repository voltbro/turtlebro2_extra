# turtlebro_py Arduino Firmware

## One-step build

From the workspace root run:

```bash
scripts/arduino_cli.sh compile -b arduino:sam:arduino_due_x_dbg src/turtlebro_extra/turtlebro_py/ino/turtlebro_py/turtlebro_py.ino
```

The helper script takes care of:

1. Mirroring your global `~/.arduino15` toolchain into `.arduino15_local/` so we can write patched files inside the workspace sandbox.
2. Applying the micro-ROS `compiler.libraries.ldflags` patch that the Arduino SAM core needs to link the precompiled micro-ROS library.
3. Pointing `arduino-cli` at the vendored sketchbook under `arduino_sketchbook/` that already includes the required third-party libraries (FastLED, Servo, Adafruit AMG88xx + deps, micro_ros_arduino).
4. Using `.cache/` for all intermediate build artifacts so the real home directory stays untouched.

After the first invocation (which copies the toolchain once) subsequent builds are incremental and only take a couple of seconds.

> Tip: if you want the plain `arduino-cli` command to pick up this wrapper automatically (so `arduino-cli compile ...` works without extra flags), run `alias arduino-cli="$PWD/scripts/arduino_cli.sh"` in your shell startup script or in the current terminal session.

## Exporting standalone HEX/BIN artifacts

Use the helper below whenever you need firmware that can be flashed on another PC without setting up the workspace:

```bash
scripts/build_arduino_hex.sh
```

The script wraps the same toolchain bootstrap as `arduino_cli.sh`, performs the build with `--export-binaries`, and then converts the resulting ELF into Intel HEX format via the ARM objcopy tool. Artifacts end up under `build/arduino_artifacts/arduino_sam_arduino_due_x_dbg/`:

- `turtlebro_py.arduino_sam_arduino_due_x_dbg.bin` — raw BOSSA-friendly binary.
- `turtlebro_py.arduino_sam_arduino_due_x_dbg.hex` — portable HEX that you can copy to another machine and flash directly.

Use `--board`, `--sketch`, `--build-dir`, or `--artifact-dir` if you need different targets/locations, e.g. `scripts/build_arduino_hex.sh --board arduino:sam:arduino_due_x --clean`.

## Updating bundled libraries

If you install a newer upstream version of any Arduino library under `~/Arduino/libraries/`, just re-run the copy step:

```bash
scripts/sync_arduino_libs.sh
```

The script overwrites the checked-in copies in `arduino_sketchbook/libraries/` so CI and teammates share the same versions.
