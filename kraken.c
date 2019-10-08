/*
 * kraken.c - NZXT Kraken X52 water cooler driver
 *
 * Copyright (c) 2019 Doug Gale <doug16k@gmail.com>
 *
 *
 * This driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this driver; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

// NZXT Kraken X52 120mm Liquid Cooler
#define VENDOR_ID   0x1E71
#define PRODUCT_ID  0x170E

static const struct usb_device_id id_table[] = {
    { USB_DEVICE(VENDOR_ID, PRODUCT_ID) },
    { }
};

MODULE_DEVICE_TABLE(usb, id_table);

struct kraken_setfan {
    u8 header[4];       // 02 4d 00 00
    u8 fan_percent;
    u8 zero[59];
};

struct kraken_setpump {
    u8 header[4];       // 02 4d 40 00
    u8 pump_percent;
    u8 zero[59];
};

struct kraken_status {
    u8 header;          // 04
    u8 liquid_temp;
    u8 unknown_1;       // 03 or 02 or possibly other values
    u8 fan_rpm[2];      // big-endian
    u8 pump_rpm[2];     // big-endian
    u8 unknown_2[10];   // 00 00 00 3f 02 00 01 08 1e 00
};

struct usb_kraken {
    struct usb_device *udev;
    struct usb_interface *interface;
    struct device *hmon_dev;

    struct hrtimer update_timer;
    struct workqueue_struct *update_workqueue;
    struct work_struct update_work;

    struct kraken_setpump setpump_msg;
    struct kraken_setfan setfan_msg;
    struct kraken_status status_msg;

    // 0=full speed, 1=manual override, 2=automatic
    int pump_enable;
    int fan_enable;

    // Limits
    int pump_min;
    int pump_max;
    int fan_min;
    int fan_max;
};

static int kraken_send_message(struct usb_kraken *kraken,
                               u8 *message, int length)
{
    int sent = 0;
    int retval = usb_bulk_msg(kraken->udev,
                              usb_sndintpipe(kraken->udev, 1),
                              message, length, &sent, 3000);

    if (unlikely(sent != length)) {
        dev_warn_ratelimited(
                    &kraken->udev->dev,
                    "USB bulk send expected to send %d, but sent %d bytes\n",
                    length, sent);
        return -EIO;
    }

    return retval;
}

static int kraken_receive_message(struct usb_kraken *kraken,
                                  u8 *message, int expected_length)
{
    int received = 0;
    int retval;

    memset(message, 0, expected_length);

    retval = usb_bulk_msg(kraken->udev,
                              usb_rcvintpipe(kraken->udev, 0x81),
                              message, expected_length, &received, 3000);

    if (unlikely(received != expected_length)) {
        dev_warn_ratelimited(
                    &kraken->udev->dev,
                    "USB bulk receive expected %d, but got %d bytes\n",
                    expected_length, received);
    }

    return retval;
}

enum attr_index {
    // RW
    idx_pump_pwm,
    idx_fan_pwm,
    idx_pump_enable,
    idx_fan_enable,

    // RO
    idx_pump_rpm,
    idx_fan_rpm,
    idx_liquid_temp,

    // Validation
    idx_max
};

static const char *attr_labels[] = {
    // RW
    [idx_pump_pwm]      = "pump_pwm",
    [idx_fan_pwm]       = "fan_pwm",
    [idx_pump_enable]   = "pump_enable",
    [idx_fan_enable]    = "fan_enable",

    // RO
    [idx_pump_rpm]      = "pump_rpm",
    [idx_fan_rpm]       = "fan_rpm",
    [idx_liquid_temp]   = "liquid_temp"
};

struct attr_range {
    int min_value;
    int max_value;
};

static const struct attr_range attr_ranges[] = {
    [idx_pump_pwm]      = { 0, 0xFF },
    [idx_fan_pwm]       = { 0, 0xFF },
    [idx_pump_enable]   = { 0, INT_MAX },
    [idx_fan_enable]    = { 0, INT_MAX }
};

enum kraken_channel_idx {
    kraken_channel_pump,
    kraken_channel_fan
};

static int kraken_update(struct usb_kraken *kraken)
{
    int retval = 0;

    if (kraken->pump_enable == 0)
        kraken->setpump_msg.pump_percent = 100;

    if (kraken->fan_enable == 0)
        kraken->setfan_msg.fan_percent = 100;

    if (kraken->pump_enable >= 2 || kraken->fan_enable >= 2) {
        // The range of the liquid temperature that
        // corresponds to minimum and maximum cooling
        const int min_lt = 29;
        const int max_lt = 35;
        const int rng_lt = max_lt - min_lt;

        int ofs_lt = kraken->status_msg.liquid_temp - min_lt;

        // Out of range values linearly extrapolate

        if (kraken->pump_enable >= 2) {
            // Automatic pump speed

            // The pump throttle range
            // When liquid temp is min_lt, set pump throttle to 45%
            // When liquid temp is max_lt, set pump throttle to 85%
            const int min_pt = 45;
            const int max_pt = 85;
            const int rng_pt = max_pt - min_pt;

            int pt = ((ofs_lt * rng_pt) / rng_lt) + min_pt;

            if (pt < kraken->pump_min)
                pt = kraken->pump_min;
            if (pt > kraken->pump_max)
                pt = kraken->pump_max;

            dev_dbg_ratelimited(&kraken->udev->dev,
                                 "Auto adjusted %s to %d%%\n",
                                 attr_labels[idx_pump_pwm], pt);

            kraken->setpump_msg.pump_percent = pt;
        }

        if (kraken->fan_enable >= 2) {
            // Automatic fan speed

            // The fan throttle range
            // When liquid temp is min_lt, set fan throttle to 25%
            // When liquid temp is max_lt, set fan throttle to 100%
            const int min_ft = 33;
            const int max_ft = 100;
            const int rng_ft = max_ft - min_ft;

            int ft = ((ofs_lt * rng_ft) / rng_lt) + min_ft;

            if (ft < kraken->fan_min)
                ft = kraken->fan_min;
            if (ft > kraken->fan_max)
                ft = kraken->fan_max;

            kraken->setfan_msg.fan_percent = ft;

            dev_dbg_ratelimited(&kraken->udev->dev,
                                "Auto adjusted %s to %d%%\n",
                                attr_labels[idx_fan_pwm], ft);
        }
    }

    dev_dbg_ratelimited(&kraken->udev->dev, "Writing %d%% to pump hardware",
                        kraken->setpump_msg.pump_percent);

    retval = kraken_send_message(kraken, (u8*)&kraken->setpump_msg,
                                 sizeof(kraken->setpump_msg));
    if (retval < 0)
        goto send_failed;

    retval = kraken_receive_message(kraken, (u8*)&kraken->status_msg,
                                    sizeof(kraken->status_msg));
    if (retval < 0)
        goto receive_failed;

    dev_dbg_ratelimited(&kraken->udev->dev, "Writing %d%% to fan hardware",
                        kraken->setfan_msg.fan_percent);

    retval = kraken_send_message(kraken, (u8*)&kraken->setfan_msg,
                                 sizeof(kraken->setfan_msg));
    if (retval < 0)
        goto send_failed;

    retval = kraken_receive_message(kraken, (u8*)&kraken->status_msg,
                                    sizeof(kraken->status_msg));
    if (retval < 0)
        goto receive_failed;

    return 0;

send_failed:
    dev_err(&kraken->udev->dev, "Failed to send update: %d\n", retval);
    return retval;

receive_failed:
    dev_err(&kraken->udev->dev, "Failed to receive: %d\n", retval);
    return retval;
}

static u16 be16_bytes(const u8 *bytes)
{
    return (bytes[0] << 8) | bytes[1];
}

static umode_t kraken_is_visible(
        const void *drvdata, enum hwmon_sensor_types type,
        u32 attr, int channel)
{
    switch (type) {
    case hwmon_temp:
        switch (attr) {
        case hwmon_temp_input:
        case hwmon_temp_label:
            return 0444;

        }
        break;

    case hwmon_pwm:
        switch (attr) {
        case hwmon_pwm_input:
        case hwmon_pwm_enable:
            return 0644;

        }
        break;

    case hwmon_fan:
        switch (attr) {
        case hwmon_fan_input:
        case hwmon_fan_label:
            switch (channel) {
            case kraken_channel_pump:
            case kraken_channel_fan:
                return 0444;

            }
            break;

        }
        break;

    default:
        return 0;

    }

    return 0;
}

static int kraken_read_string(struct device *dev, enum hwmon_sensor_types type,
                              u32 attr, int channel, const char **str)
{
    switch (type) {
    case hwmon_temp:
        switch (attr) {
        case hwmon_temp_label:
            *str = "liquid";
            return 0;
        }
        break;

    case hwmon_fan:
        switch (attr) {
        case hwmon_fan_label:
            switch (channel) {
            case kraken_channel_pump:
                *str = "pump";
                return 0;

            case kraken_channel_fan:
                *str = "fan";
                return 0;

            }
            break;

        }
        break;

    default:
        return -ENOTSUPP;

    }

    return -ENOTSUPP;
}

static int kraken_read(struct device *dev, enum hwmon_sensor_types type,
                       u32 attr, int channel, long *val)
{
    struct usb_kraken *kraken = dev_get_drvdata(dev);

    switch (type) {
    case hwmon_temp:
        dev_info(&kraken->udev->dev, "reading temp, attr=%d, channel=%d\n",
                 attr, channel);
        *val = kraken->status_msg.liquid_temp * 1000;
        return 0;

    case hwmon_pwm:
        dev_info(&kraken->udev->dev, "reading pwm, attr=%d, channel=%d\n",
                 attr, channel);

        switch (channel) {
        case kraken_channel_pump:
            switch (attr) {
            case hwmon_pwm_input:
                *val = 255 * kraken->setpump_msg.pump_percent / 100;
                return 0;

            case hwmon_pwm_enable:
                *val = kraken->pump_enable;
                return 0;

            }
            break;

        case kraken_channel_fan:
            switch (attr) {
            case hwmon_pwm_input:
                *val = 255 * kraken->setfan_msg.fan_percent / 100;
                return 0;

            case hwmon_pwm_enable:
                *val = kraken->fan_enable;
                return 0;

            }
            break;

        }
        break;

    case hwmon_fan:
        dev_info(&kraken->udev->dev, "reading fan, attr=%d, channel=%d\n",
                 attr, channel);

        switch (channel) {
        case kraken_channel_pump:
            *val = be16_bytes(kraken->status_msg.pump_rpm);
            return 0;

        case kraken_channel_fan:
            *val = be16_bytes(kraken->status_msg.fan_rpm);
            return 0;
        }
        break;

    default:
        dev_info(&kraken->udev->dev, "reading unknown, attr=%d, channel=%d\n",
                 attr, channel);
        break;

    }

    return -ENOTSUPP;
}

static int kraken_write(struct device *dev, enum hwmon_sensor_types type,
                       u32 attr, int channel, long val)
{
    struct usb_kraken *kraken = dev_get_drvdata(dev);

    switch (type) {
    case hwmon_pwm:
        switch (channel) {
        case kraken_channel_pump:
            kraken->setpump_msg.pump_percent = 100 * val / 255;
            return 0;

        case kraken_channel_fan:
            kraken->setfan_msg.fan_percent = 100 * val / 255;
            return 0;

        }
        break;

    default:
        return -ENOTSUPP;

    }

    return -ENOTSUPP;
}

static const struct hwmon_ops kraken_chip_ops = {
    .is_visible = kraken_is_visible,
    .read_string = kraken_read_string,
    .read = kraken_read,
    .write = kraken_write
};

static const u32 kraken_chip_config[] = {
    HWMON_C_REGISTER_TZ | HWMON_C_UPDATE_INTERVAL,
    0
};

static const u32 kraken_temp_config[] = {
    HWMON_T_INPUT | HWMON_T_LABEL,
    0
};

static const u32 kraken_pwm_config[] = {
    [kraken_channel_pump] = HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
    [kraken_channel_fan]  = HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
    0
};

static const u32 kraken_fan_config[] = {
    [kraken_channel_pump] = HWMON_F_INPUT | HWMON_F_LABEL,
    [kraken_channel_fan]  = HWMON_F_INPUT | HWMON_F_LABEL,
    0
};

static const struct hwmon_channel_info kraken_chip = {
    .type = hwmon_chip,
    .config = kraken_chip_config
};

static const struct hwmon_channel_info kraken_temp = {
    .type = hwmon_temp,
    .config = kraken_temp_config
};

static const struct hwmon_channel_info kraken_pwm = {
    .type = hwmon_pwm,
    .config = kraken_pwm_config
};

static const struct hwmon_channel_info kraken_fan = {
    .type = hwmon_fan,
    .config = kraken_fan_config
};

static const struct hwmon_channel_info *kraken_channels[] = {
    &kraken_temp,
    &kraken_pwm,
    &kraken_fan,
    NULL
};

static const struct hwmon_chip_info kraken_chip_info = {
    .ops = &kraken_chip_ops,
    .info = kraken_channels
};

static ssize_t show_device_name(
        struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", "kraken-hid-0000");
}

static DEVICE_ATTR(name, S_IRUGO, show_device_name, NULL);

void kraken_remove_device_files(void *arg)
{
    struct usb_interface *interface = arg;

    device_remove_file(&interface->dev, &dev_attr_name);
}

static int kraken_add_device_files(struct usb_interface *interface)
{
    struct usb_kraken *dev = usb_get_intfdata(interface);

    int retval = 0;

    retval = device_create_file(&interface->dev, &dev_attr_name);

    if (retval < 0)
        return retval;

    retval = devm_add_action_or_reset(&interface->dev,
                             kraken_remove_device_files, interface);

    if (unlikely(retval < 0))
        return retval;

    dev->hmon_dev = devm_hwmon_device_register_with_info(
                &interface->dev, "kraken", dev,
                &kraken_chip_info, NULL);

    if (IS_ERR(dev->hmon_dev))
        retval = PTR_ERR(dev->hmon_dev);

    return retval;
}

static enum hrtimer_restart kraken_update_timer(struct hrtimer *update_timer)
{
    struct usb_kraken *dev = container_of(
                update_timer, struct usb_kraken, update_timer);
    queue_work(dev->update_workqueue, &dev->update_work);
    hrtimer_forward(update_timer, ktime_get(), ktime_set(1, 0));
    return HRTIMER_RESTART;
}

static void kraken_update_work(struct work_struct *param)
{
    struct usb_kraken *dev = container_of(
                param, struct usb_kraken, update_work);
    kraken_update(dev);
}

void kraken_cleanup_timer(struct usb_kraken *dev)
{
    flush_workqueue(dev->update_workqueue);
    destroy_workqueue(dev->update_workqueue);
    hrtimer_cancel(&dev->update_timer);
}

static int kraken_probe(struct usb_interface *interface,
                        const struct usb_device_id *id)
{
    struct usb_device *udev;
    struct usb_kraken *dev;
    int retval;
    u8 *descriptor;

    const size_t descriptor_size = 130;

    udev = interface_to_usbdev(interface);
    dev = NULL;
    retval = -ENOMEM;

    dev = devm_kzalloc(&interface->dev, sizeof(*dev), GFP_KERNEL);
    if (unlikely(!dev))
        goto error_dev;

    // Default to automatic
    dev->pump_enable = 2;
    dev->fan_enable = 2;

    // Default limits (the OEM software never send a percentage less than 30)
    dev->pump_min = 30;
    dev->fan_min = 30;
    dev->pump_max = 100;
    dev->fan_max = 100;

    dev->setfan_msg.header[0] = 0x02;
    dev->setfan_msg.header[1] = 0x4d;
    dev->setfan_msg.header[2] = 0x00;
    dev->setfan_msg.header[3] = 0x00;
    dev->setfan_msg.fan_percent = 60;

    dev->setpump_msg.header[0] = 0x02;
    dev->setpump_msg.header[1] = 0x4d;
    dev->setpump_msg.header[2] = 0x40;
    dev->setpump_msg.header[3] = 0x00;
    dev->setpump_msg.pump_percent = 60;

    dev->udev = usb_get_dev(udev);
    usb_set_intfdata(interface, dev);

    descriptor = devm_kcalloc(&interface->dev, 1, descriptor_size, GFP_KERNEL);

    if (unlikely(!descriptor))
        return -ENOMEM;

    // Device seems to require reading the descriptor from the control pipe
    // Make it happy
    retval = usb_control_msg(udev,
                             usb_sndctrlpipe(udev, 0),
                             6, 0x80, 0x0303, 0x409,
                             descriptor, descriptor_size, 1000);
    devm_kfree(&interface->dev, descriptor);
    descriptor = NULL;

    if (retval < 0) {
        dev_err(&interface->dev, "Error sending initial control message: %d\n",
                retval);
        goto error;
    }

    //
    // Initialize polling timer driven work function

    hrtimer_init(&dev->update_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

    dev->update_timer.function = kraken_update_timer;
    hrtimer_start(&dev->update_timer,
                  ktime_set(0, 500000000), HRTIMER_MODE_REL);

    dev->update_workqueue = create_singlethread_workqueue("kraken_up");
    INIT_WORK(&dev->update_work, kraken_update_work);

    if (unlikely(!dev->update_workqueue))
        goto error_work_queue;

    // Initialize hwmon device
    if (unlikely((retval = kraken_add_device_files(interface)) < 0))
        goto error_device_files;

    dev_info(&interface->dev, "Kraken connected\n");
    return 0;

error_device_files:
    kraken_cleanup_timer(dev);

error_work_queue:

error:
    usb_set_intfdata(interface, NULL);
    usb_put_dev(dev->udev);

error_dev:
    return retval;
}

static void kraken_disconnect(struct usb_interface *interface)
{
    struct usb_kraken *dev = usb_get_intfdata(interface);

    usb_set_intfdata(interface, NULL);
    usb_put_dev(dev->udev);

    kraken_cleanup_timer(dev);

    dev_info(&interface->dev, "Kraken disconnected\n");
}

static struct usb_driver kraken_driver = {
    .name       = "kraken",
    .probe      = kraken_probe,
    .disconnect = kraken_disconnect,
    .id_table   = id_table,
};

module_usb_driver(kraken_driver);

MODULE_SUPPORTED_DEVICE("Kraken X52 240mm liquid cooler");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("NZXT Kraken X52 driver");
MODULE_AUTHOR("Doug Gale <doug16k@gmail.com>");
MODULE_LICENSE("GPL");
