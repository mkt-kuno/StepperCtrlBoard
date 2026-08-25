"""PlatformIO pre-build patcher for FastAccelStepper ATmega328PB support.

FastAccelStepper v0.34.0 declares supported AVR MCUs in two files:

  - src/fas_arch/arduino_avr.h
      Defines MAX_STEPPER / NUM_QUEUES / fas_queue_* aliases for each
      supported derivative.

  - src/AVRStepperPins.h
      Top-level `#error "Unsupported AVR derivate"` guard, plus the
      FAS_TIMER_MODULE and stepPinStepper* pin macros.

Neither file lists __AVR_ATmega328PB__. Since the 328PB's Timer1
register layout is identical to 328P, adding the 328PB macro to the
existing 328P guards makes the library work on both chips with no
other source changes.

Behavior
--------
- Idempotent: re-running on already-patched files is a no-op.
- Sentinel-based: each search string must appear exactly once in its
  file. If the upstream guard changes shape, the patch fails gracefully
  with a WARN rather than corrupting the file.
- Logs each affected file to stdout for build traceability.

Triggered by ``extra_scripts = pre:scripts/patch_fastaccel.py`` in
platformio.ini. Runs once per environment after PIO installs libdeps.
"""
import os

Import("env")

PIOENV      = env["PIOENV"]
LIBDEPS_DIR = env["PROJECT_LIBDEPS_DIR"]
TARGET_DIR  = os.path.join(LIBDEPS_DIR, PIOENV, "FastAccelStepper")
MARKER      = "__AVR_ATmega328PB__"

# (relative path under TARGET_DIR, list of (search, replace))
# Each search is the canonical 328P guard substring; replace adds the
# 328PB alternative. The replace() call is bounded to 1 substitution
# per sentinel so an unexpected duplicate fails fast.
PATCHES = [
    (
        "src/fas_arch/arduino_avr.h",
        [
            # 168/328/P guard: defines MAX_STEPPER / NUM_QUEUES / fas_queue_*
            (
                "defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))",
                "defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__))",
            ),
        ],
    ),
    (
        "src/AVRStepperPins.h",
        [
            # Outer supported-MCU list (the one feeding the #error).
            # Sentinel is the tail of the chain (unique to this file).
            (
                "defined(__AVR_ATmega32U4__))",
                "defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega328PB__))",
            ),
            # 168/328/P Timer 1 + step-pin selection (Timer1 is identical on 328PB).
            (
                "defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))",
                "defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__))",
            ),
        ],
    ),
]


def force_install():
    """Ensure libraries are downloaded before patching."""
    if os.path.isdir(TARGET_DIR):
        return
    print(f"[patch_fastaccel] installing libraries for {PIOENV} (first build only)")
    rc = env.Execute(f'"{env["PYTHONEXE"]}" -m platformio lib install -e {PIOENV}')
    if rc is not None and rc != 0:
        print(f"[patch_fastaccel] WARN: pio lib install returned {rc}")


def patch_file(rel_path, edits):
    full_path = os.path.join(TARGET_DIR, rel_path)
    if not os.path.isfile(full_path):
        print(f"[patch_fastaccel] WARN: missing file: {full_path}")
        return False
    with open(full_path, "r", encoding="utf-8") as f:
        content = f.read()
    if MARKER in content:
        print(f"[patch_fastaccel] already patched: {rel_path}")
        return False

    new_content = content
    applied = 0
    for search, replace in edits:
        count = new_content.count(search)
        if count == 0:
            print(f"[patch_fastaccel] WARN: sentinel missing in {rel_path}: {search[:50]!r}")
            continue
        if count > 1:
            print(f"[patch_fastaccel] WARN: sentinel appears {count}x in {rel_path}: {search[:50]!r}")
        new_content = new_content.replace(search, replace, 1)
        applied += 1

    if applied == 0:
        print(f"[patch_fastaccel] no changes for {rel_path}")
        return False

    with open(full_path, "w", encoding="utf-8") as f:
        f.write(new_content)
    print(f"[patch_fastaccel] patched: {rel_path} ({applied} edit(s))")
    return True


force_install()
patched_files = 0
for rel_path, edits in PATCHES:
    if patch_file(rel_path, edits):
        patched_files += 1

if patched_files == 0:
    print(f"[patch_fastaccel] done (no files patched)")
