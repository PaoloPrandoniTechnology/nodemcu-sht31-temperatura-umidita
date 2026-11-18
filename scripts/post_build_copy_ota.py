import os
import re
import shutil
from datetime import datetime

Import("env")


def read_fw_version_from_source(project_dir):
    main_cpp = os.path.join(project_dir, "src", "main.cpp")
    try:
        with open(main_cpp, "r", encoding="utf-8") as f:
            src = f.read()
        m = re.search(r'#define\s+FW_VERSION_STR\s+"([^"]+)"', src)
        if m:
            return m.group(1)
    except Exception:
        pass
    return "0.0.0"


def sanitize_name(s):
    return re.sub(r"[^\w.\-]+", "_", s).strip("_")


def copy_firmware_post_build(source, target, env):
    project_dir = env.subst("$PROJECT_DIR")
    build_dir = env.subst("$BUILD_DIR")
    pio_env = env.subst("$PIOENV")

    src_bin = os.path.join(build_dir, "firmware.bin")
    if not os.path.isfile(src_bin):
        print("[OTA-COPY] ATTENZIONE: firmware.bin non trovato in %s" % src_bin)
        return

    out_dir = os.path.join(project_dir, "firmware_ota")
    try:
        os.makedirs(out_dir, exist_ok=True)
    except Exception:
        pass

    version = sanitize_name(read_fw_version_from_source(project_dir))
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    latest_bin = os.path.join(out_dir, "latest_firmware.bin")
    versioned_bin = os.path.join(
        out_dir, "firmware_v%s_%s_%s.bin" % (version, sanitize_name(pio_env), ts)
    )

    shutil.copy2(src_bin, latest_bin)
    shutil.copy2(src_bin, versioned_bin)

    size_src = os.path.getsize(src_bin)
    size_latest = os.path.getsize(latest_bin)
    size_ver = os.path.getsize(versioned_bin)

    print("")
    print("================================================================")
    print(" OTA FIRMWARE OUTPUT (cartella pronta per upload via scheda Web)")
    print("================================================================")
    print("  Cartella      : %s" % out_dir)
    print("  File latest   : %s  (%d bytes)" % (os.path.basename(latest_bin), size_latest))
    print("  File versionato: %s  (%d bytes)" % (os.path.basename(versioned_bin), size_ver))
    print("  Sorgente build: %s  (%d bytes)" % (src_bin, size_src))
    print("================================================================")
    print("")


env.AddPostAction("$BUILD_DIR/firmware.bin", copy_firmware_post_build)
