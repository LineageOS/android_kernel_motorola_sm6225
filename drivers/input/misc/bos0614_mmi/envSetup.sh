#To configure
export ANDROID_AOSP="/home/pfstlaurent/workspace/HDK855_Android-P_v1.0/Source_Package/SM8150P_HDK_855_Android-P_v1.0/"
export ANDROID_KERNEL_LOC=$ANDROID_AOSP/out/target/product/msmnile/obj/kernel/msm-4.14

CURRENT_PATH=$(pwd)
cd $ANDROID_AOSP
. build/envsetup.sh
lunch msmnile-userdebug
cd $CURRENT_PATH

export CROSS_COMPILE_ARM32=$ANDROID_AOSP/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin
export CLANG=$ANDROID_AOSP/prebuilts/clang/host/linux-x86/clang-4691093/bin
export PATH=$PATH:$CROSS_COMPILE_ARM32:$CLANG
export TARGET_PREBUILT_KERNEL=$ANDROID_KERNEL_LOC/out/android-msm-pixel-4.9/private/msm-google/arch/arm64/boot/Image.lz4-dtb

adb root
alias build="ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- make CC=clang KERNELDIR=$ANDROID_KERNEL_LOC"
alias push="adb push bosDriverModule.ko /mnt/vendor/persist"
alias load="adb shell insmod /mnt/vendor/persist/bosDriverModule.ko"
alias rmmod="adb shell rmmod bosDriverModule.ko"
alias lsmod="adb shell lsmod"
alias new_device="adb shell \"echo bos0614 0x2c > /sys/bus/i2c/devices/i2c-0/new_device\""
alias new_device1="adb shell \"echo bos0614 0x2c > /sys/bus/i2c/devices/i2c-1/new_device\""
alias new_device2="adb shell \"echo bos0614 0x2c > /sys/bus/i2c/devices/i2c-2/new_device\""
alias del_device="adb shell \"echo bos0614 0x2c > /sys/bus/i2c/devices/i2c-0/delete_device\""
alias del_device1="adb shell \"echo bos0614 0x2c > /sys/bus/i2c/devices/i2c-1/delete_device\""
alias del_device2="adb shell \"echo bos0614 0x2c > /sys/bus/i2c/devices/i2c-2/delete_device\""
alias status="adb shell \"cat /sys/bus/i2c/devices/1-002c/ic_errors\""
alias pressSensing="adb shell \"echo 0 0 0 4000 1200 4 > /sys/bus/i2c/devices/1-002c/sensing_config\""
alias pressAutoplay="adb shell \"echo 0 0 0 > /sys/bus/i2c/devices/1-002c/sensing_autoplay\""
alias releaseSensing="adb shell \"echo 0 1 1 8000 -11 32 > /sys/bus/i2c/devices/1-002c/sensing_config\""
alias releaseAutoplay="adb shell \"echo 0 1 1 > /sys/bus/i2c/devices/1-002c/sensing_autoplay\""
alias pressStop="adb shell \"echo 0 0 > /sys/bus/i2c/devices/1-002c/sensing_stop\""
alias releaseStop="adb shell \"echo 0 1 > /sys/bus/i2c/devices/1-002c/sensing_stop\""
alias log="adb shell dmesg -T -w"
alias sliceA="adb shell \"echo 0 60000 175000 2 1 > /sys/bus/i2c/devices/1-002c/set_slice\""
alias sliceStabA="adb shell \"echo 1 00000 250000 2 1 > /sys/bus/i2c/devices/1-002c/set_slice\""
alias waveformA="adb shell \"echo 0 0 2 1 1 > /sys/bus/i2c/devices/1-002c/set_waveform\""
alias sliceB="adb shell \"echo 2 60000 175000 2 1 > /sys/bus/i2c/devices/1-002c/set_slice\""
alias sliceStabB="adb shell \"echo 3 00000 250000 11 1 > /sys/bus/i2c/devices/1-002c/set_slice\""
alias waveformB="adb shell \"echo 1 2 2 1 1 > /sys/bus/i2c/devices/1-002c/set_waveform\""
alias sensingOn="sliceA && sliceStabA &&waveformA && sliceB && sliceStabB && waveformB && pressSensing && pressAutoplay && releaseSensing && releaseAutoplay"
alias sensingOff="pressStop && releaseStop"
alias play="adb shell \"echo 0 0 > /sys/bus/i2c/devices/1-002c/synth_play\""
alias output_on="adb shell \"echo 1 > /sys/bus/i2c/devices/1-002c/ctrl_output\""
alias output_off="adb shell \"echo 0 > /sys/bus/i2c/devices/1-002c/ctrl_output\""
alias demoWave="sliceA && waveform && play && output_on"
alias refresh="(push & rmmod) && load"
alias test="build && refresh"
