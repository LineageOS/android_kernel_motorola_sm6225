#To configure device i2c bus
i2c_bus="i2c-2"
device="2-002c"

adb root

alias new_device="adb shell \"echo bos0614 0x2c > /sys/bus/i2c/devices/$i2c_bus/new_device\""
alias del_device="adb shell \"echo bos0614 0x2c > /sys/bus/i2c/devices/$i2c_bus/delete_device\""

alias status="adb shell \"cat /sys/bus/i2c/devices/$device/ic_errors\""
alias pressSensing="adb shell \"echo 0 0 0 4000 1200 25 > /sys/bus/i2c/devices/$device/sensing_config\""
alias pressAutoplay="adb shell \"echo 0 0 0 > /sys/bus/i2c/devices/$device/sensing_autoplay\""
alias releaseSensing="adb shell \"echo 0 1 1 8000 -11 32 > /sys/bus/i2c/devices/$device/sensing_config\""
alias releaseAutoplay="adb shell \"echo 0 1 1 > /sys/bus/i2c/devices/$device/sensing_autoplay\""
alias deactivateButton="adb shell \"echo 0  > /sys/bus/i2c/devices/$device/sensing_stop\""

alias sliceA="adb shell \"echo 0 60000 175000 2 1 > /sys/bus/i2c/devices/$device/set_slice\""
alias waveformA="adb shell \"echo 0 0 2 1 1 > /sys/bus/i2c/devices/$device/set_waveform\""
alias sliceB="adb shell \"echo 2 60000 175000 2 1 > /sys/bus/i2c/devices/$device/set_slice\""
alias waveformB="adb shell \"echo 1 2 2 1 1 > /sys/bus/i2c/devices/$device/set_waveform\""
alias sensingOn="sliceA &&waveformA && sliceB && waveformB && pressSensing && pressAutoplay && releaseSensing && releaseAutoplay"
alias sensingOff="deactivateButton"
alias play="adb shell \"echo 0 0 > /sys/bus/i2c/devices/$device/synth_play\""
alias output_on="adb shell \"echo 1 > /sys/bus/i2c/devices/$device/ctrl_output\""
alias output_off="adb shell \"echo 0 > /sys/bus/i2c/devices/$device/ctrl_output\""
alias demoWave="sliceA && waveform && play && output_on"

alias refresh="(push & rmmod) && load"
alias logdebug="adb shell \"echo 8 > /proc/sys/kernel/printk\""
