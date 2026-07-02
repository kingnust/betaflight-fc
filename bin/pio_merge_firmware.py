Import("env")

import subprocess
import shutil
import os
from datetime import datetime

APP_BIN = "$BUILD_DIR/${PROGNAME}.bin"
MERGED_BIN = "$BUILD_DIR/${PROGNAME}_0x00.bin"
BOARD_CONFIG = env.BoardConfig()


def merge_bin(source, target, env):
    # The list contains all extra images (bootloader, partitions, eboot) and
    # the final application binary
    flash_images = env.Flatten(env.get("FLASH_EXTRA_IMAGES", [])) + ["$ESP32_APP_OFFSET", APP_BIN]

    # Run esptool to merge images into a single binary
    cmd = [
        env.subst("$PYTHONEXE"),
        env.subst("$OBJCOPY"),
        "--chip",
        BOARD_CONFIG.get("build.mcu", "esp32"),
        "merge_bin",
        "--flash_size",
        BOARD_CONFIG.get("upload.flash_size", "4MB"),
        "-o",
        env.subst(MERGED_BIN),
    ] + [env.subst(str(image)) for image in flash_images]
    subprocess.check_call(cmd)

    merged_bin = env.subst(MERGED_BIN)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    timestamped_bin = env.subst(f"$BUILD_DIR/${{PROGNAME}}_0x00_{timestamp}.bin")
    shutil.copy2(merged_bin, timestamped_bin)
    print(f"Timestamped firmware: {timestamped_bin}")

    archive_dir = os.path.join(env.subst("$PROJECT_DIR"), "firmware_builds", env.subst("$PIOENV"))
    os.makedirs(archive_dir, exist_ok=True)
    archived_bin = os.path.join(archive_dir, f"{env.subst('${PROGNAME}')}_0x00_{timestamp}.bin")
    shutil.copy2(merged_bin, archived_bin)
    print(f"Archived firmware: {archived_bin}")

# Add a post action that runs esptoolpy to merge available flash images
env.AddPostAction(APP_BIN , merge_bin)

# Patch the upload command to flash the merged binary at address 0x0
#env.Replace(
#    UPLOADERFLAGS=[
#            f
#            for f in env.get("UPLOADERFLAGS")
#            if f not in env.Flatten(env.get("FLASH_EXTRA_IMAGES"))
#        ]
#        + ["0x0", MERGED_BIN],
#    UPLOADCMD='"$PYTHONEXE" "$UPLOADER" $UPLOADERFLAGS',
#)
