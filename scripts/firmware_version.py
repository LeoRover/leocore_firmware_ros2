import subprocess

Import("env")

ret = subprocess.run(["git", "describe", "--tags"], stdout=subprocess.PIPE, text=True)
build_version = ret.stdout.strip()
build_flag = "-D FIRMWARE_VERSION=\\\"" + build_version + "\\\""

print ("Firmware Revision: " + build_version)

env.Append(
    BUILD_FLAGS=[build_flag]
)