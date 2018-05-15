# Kraken X52 driver for linux

This driver is a heavily rewritten fork based on
[https://github.com/jaksi/leviathan](https://github.com/jaksi/leviathan).

Linux device driver that supports controlling and monitoring
NZXT Kraken water coolers

NZXT is **NOT** involved in this project, do **NOT** contact them if your
device is damaged while using this software.

Also, while it doesn't seem like the hardware could be damaged by silly USB
messages (apart from overheating), I do **NOT** take any responsibility for
any damage done to your cooler.

# Supported devices
* NZXT Kraken X52 (Vendor/Product ID: `1e71:170e`)
(Only for controlling the fan/pump speed, I did not bother with control of RGB)

# Installation
Make sure the headers for the kernel you are running are installed.
```Shell
make
sudo insmod kraken.ko
```

# Usage
The driver can be controlled with device files under
`/sys/bus/usb/drivers/kraken`.

Find the symbolic links that point to the connected compatible devices.

In my case, there's only one Kraken connected:
```Shell
/sys/bus/usb/drivers/kraken/1-8:1.0
```

1-8:1.0 may vary depending on your mainboard and which connector is used.
It is referred to as `DEVICE` below.

# USB HID

The usbhid driver may have seized control of the device. You may need to
unbind it from usbhid and bind it to the kraken driver. To do this, you
need to `echo DEVICE | sudo tee /sys/bus/usb/drivers/usbhid/unbind`, then
`echo DEVICE | sudo tee /sys/bus/usb/drivers/kraken/bind`

## Automatic control

You can enable automatic control of the pump and fan based on the liquid
temperature. It runs the pump and fan at 50% at 28C, and ramps up linearly
to 85% at 39C. On my system, this holds the CPU temperature below 60C at
full load, and ramps the fan and pump down to practically silent at idle.

## Enabling automatic control

echo '1' | sudo tee /sys/bus/usb/drivers/kraken/DEVICE/auto_throttle

## Disabling automatic control

echo '0' | sudo tee /sys/bus/usb/drivers/kraken/DEVICE/auto_throttle

## Reading the liquid temperature

```Shell
cat /sys/bus/usb/drivers/kraken/DEVICE/liquid_temp
```

## Reading the pump RPM

```Shell
cat /sys/bus/usb/drivers/kraken/DEVICE/pump_rpm
```

## Reading the fan RPM

```Shell
cat /sys/bus/usb/drivers/kraken/DEVICE/fan_rpm
```

## Reading the current pump throttle

```Shell
cat /sys/bus/usb/drivers/kraken/DEVICE/pump_throttle
```

## Reading the current fan throttle

```Shell
cat /sys/bus/usb/drivers/kraken/DEVICE/fan_throttle
```

## Setting the pump throttle to 65%

```Shell
echo 65 | sudo tee /sys/bus/usb/drivers/kraken/DEVICE/pump_throttle
```

## Setting the fan throttle to 65%

```Shell
echo 65 | sudo tee /sys/bus/usb/drivers/kraken/DEVICE/fan_throttle
```
