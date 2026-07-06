Import("env")

from pathlib import Path


def patch_vl53_model_id(*_args, **_kwargs):
    libdeps_dir = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV")
    source = libdeps_dir / "VL53L1X" / "VL53L1X.cpp"
    if not source.exists():
        return

    text = source.read_text(encoding="utf-8")
    old = "  if (readReg16Bit(IDENTIFICATION__MODEL_ID) != 0xEACC) { return false; }"
    new = (
        "  const uint16_t modelId = readReg16Bit(IDENTIFICATION__MODEL_ID);\n"
        "  if (modelId != 0xEACC && modelId != 0xEAAA) { return false; }"
    )

    if new in text:
        return
    if old not in text:
        print("VL53L1X patch: model ID check pattern not found")
        return

    source.write_text(text.replace(old, new), encoding="utf-8")
    print("VL53L1X patch: accepting model IDs 0xEACC and 0xEAAA")


patch_vl53_model_id()
env.AddPreAction("buildprog", patch_vl53_model_id)
