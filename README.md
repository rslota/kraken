# Kraken X52 driver for linux

This driver is a heavily rewritten fork based on
[https://github.com/jaksi/leviathan](https://github.com/jaksi/leviathan).

Linux device driver that supports controlling and monitoring
NZXT Kraken water coolers.

NZXT is **NOT** involved in this project, do **NOT** contact them if your
device is damaged while using this software.

Also, while it doesn't seem like the hardware could be damaged by silly USB
messages (apart from overheating), I do **NOT** take any responsibility for
any damage done.

## Supported devices
* NZXT Kraken X52 (Vendor/Product ID: `1e71:170e`)
(Only for controlling the fan/pump speed, and monitoring the pump and fan rpm
and liquid temperature. I did not bother with control of RGB)

## Build

```Shell
make
make update
```

## Usage
The driver implements the standard hwmon interface.

You can find the correct hwmon index:

```Shell
grep kraken /sys/class/hwmon/hwmon*/name
```

The numbered hwmon (for example, hwmon2) is should be used for the
DEVICE placeholders below.

## USB HID

The USB HID driver may sieze control of the cooler, because the cooler
is implemented as a "hiddev", which are non-human-interface
human-interface-devices. You can't make this stuff up.

## hwmon standard names

| File        | Purpose                          | Access |
|:------------|:---------------------------------|:------:|
| pwm1        | Pump throttle                    |   RW   |
| pwm2        | Fan throttle                     |   RW   |
| pwm1_enable | 0=full, 1=manual, 2+=auto (Pump) |   RW   |
| pwm2_enable | 0=full, 1=manual, 2+=auto (Fan)  |   RW   |
| fan1_input  | Pump RPM                         |   RO   |
| fan2_input  | Fan RPM                          |   RO   |
| temp1_input | Liquid temperature               |   RO   |
| name        | "kraken"                         |   RO   |

### Automatic control

The default behaviour is to have 2 in both _enable files, resulting in
automatic speed control by default. If you want to override the automatic
control, write 1 to the appropriate _enable file from the table above.

### PWM

The values for the pwmN files are 0 to 255, corresponding to 0% to 100%.

### RPM

The values for the fanN_input files are in RPM.

### Liquid temperature

The temp1_input file is in 1/1000ths of a degree C (millicentigrade).
For example, 30000 is 30C, 35000 is 35C.

### Labels

Each file has a corresponding _label file from which you can read a
reasonably human readable description of the purpose of the
corresponding file.
