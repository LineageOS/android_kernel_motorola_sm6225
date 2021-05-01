# BOS0614 Kernel Driver

![GitHub Logo](/images/logo.png)

_Version: 1.1.0_

_Copyright (c) 2021 Boreas Technologies All rights reserved._

_GNU GENERAL PUBLIC V2 LICENSE_

Allow to configure the button press & release detection with feedback generated with the synthesizer. This linux kernel
driver provides a file system API (String base). The binary file API & GPIO configuration will be supported in the
subsequent release.

A device tree support example is available within the dts folder. The default configuration enables the button behaviour
for a specific set of channel on driver probe. This allows the system to remain in a knows state at boot up.

BOS0614 Revision. B & C are supported by this driver.

_Tested against the Linux Kernel V4.14_

## File System API (user space)

### chip_id

_Access: Read only_

Return the chip id in hexadecimal value

* 0x0D: BOS0614 Rev B

### sensing_config

_Access: Write only_

Configure the hardware press or release detection on chip

**Arguments**

* channel id :        The channel number starting from 0
* mode :              The detection mode such threshold (0) or slope (1) detection
* direction:          The button direction such press (0) or release (1)
* debouncing:         The debouncing duration in microseconds
* stabilization:      The stabilization duration in milliseconds

### sensing_autoplay

_Access: Write only_

Setup autoplay configuration. The autoplay mechanism allows the BOS0614 to play automatically a feedback once a press or
release event is detected.

**Arguments**

* channel id :        The channel number starting from 0
* waveform id:        The waveform id to play as a feedback
* direction:          The button direction which the specified feedback will be played

### sensing_stop

_Access: Write only_

Deactivate the button detection for a specific channel & direction

**Arguments**

* channel id :        The channel number starting from 0

### set_slice

_Access: Write only_

Create a slice (simple waveform). Slices are allocated into the BOS0614 RAM using the slice id. If you declare multiple
time the same slice id, only the latest set slice configuration will remain into the RAM.

**Arguments**

* slice id:           The slice id (must be unique)
* amplitude:          The sinus amplitude in milliVolt
* frequency:          The sinus frequency in milliHertz.
* cycle:              Duration of the sinus in period (only integer)
* channel mask:       The list of channel to play the waveform. Each bit represent a channel #. The LSB reprensents the
  channel #0 and the fourth bit (from LSB) the channel #4. The maximum value is 15d.

### set_waveform

_Access: Write only_

Create a waveform by composing it with slices. Same as the slice. Waveforms are allocated into the BOS0614 RAM. Waveform
unicity is based on the waveform id.

**Arguments**

* waveform id:        The waveform id
* start slice id:     The first slice of the sequence to play
* number of slice:    The number of slice to include starting from the start slice id param.
* cycle:              The amount of time the sequence of slices is repeated
* channel mask:       The list of channel to play the waveform. Each bit represent a channel #. The LSB reprensents the
  channel #0 and the fourth bit (from LSB) the channel #4. The maximum value is 15d.

### ctrl_output

_Access: Write only_

Control the BOS0614 output drive

**Arguments**

*state:               Output state: On (1) and Off (0)

### ic_errors

_Access: Read only_

Read the IC operational status. Operational status are defined in the datasheet
> SC: 0 UVLO: 0 IDAC: 0 MAX_POWER: 0 OVT: 0 OVV: 0

### synth_play

_Access: Write only_

Arm the synthesizer for playing a specific sequence of waveforms. The waveform sequence is continuous sequence.

**Arguments**

* start:              The waveform id to start with.
* stop:               The waveform id to end with (inclusively).

## Virtual Button Example - on chip detection

This example configures the button detection mechanism on the channel #0 with a specific feedback for the press and one
for the release event. This example uses an i2c device instance on the bus i2c #1 with the address 0x002c.

Note: The stabilization slice is not required anymore with this bos0614 revision C. The stabilization setting accept
values between 0 & 103 ms for the release and press event.

### 1. Configure the feedback for press event.

Press Feedback:

Press Slice (Slice #0) : 60 volts amplitude @175 Hz 2 Cycle on channel #0

`echo 0 60000 175000 2 1 > /sys/bus/i2c/devices/1-002c/set_slice`

Setup the waveform #0 using the two previous slices

`echo 0 0 2 1 1 > /sys/bus/i2c/devices/1-002c/set_waveform`

### 2. Configure the feedback for release event.

Release Slice (Slice #2) : 60 volts amplitude @170 Hz 2 Cycle on channel #0

`echo 2 60000 170000 2 1 > /sys/bus/i2c/devices/1-002c/set_slice`

Setup the waveform #1 using the two previous slices

`echo 1 2 1 1 1 >/sys/bus/i2c/devices/1-002c/set_waveform`

### 3. Activate Press Detection and Autoplay on button press event

Setup press detection with threshold mode, 1.2 Volt threshold, 4000 uS debounce time and 25 milliseconds stabilization

`echo 0 0 0 4000 1200 25 > /sys/bus/i2c/devices/1-002c/sensing_config`

`echo 0 0 0 >/sys/bus/i2c/devices/1-002c/sensing_autoplay`

### 4. Activate Release Detection and Autoplay on button release event

Setup release detection with slope mode, -11 mV gradient, 8000 uS debounce time and 32 milliseconds stabilization

`echo 0 1 1 8000 -11 32 > /sys/bus/i2c/devices/1-002c/sensing_config`

`echo 0 1 1 > /sys/bus/i2c/devices/1-002c/sensing_autoplay`




