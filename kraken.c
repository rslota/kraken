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
    u8 zero[58];
};

struct kraken_setpump {
    u8 header[4];       // 02 4d 40 00
    u8 pump_percent;
    u8 zero[58];
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

    struct hrtimer update_timer;
    struct workqueue_struct *update_workqueue;
    struct work_struct update_work;

    struct kraken_setfan setfan_msg;
    struct kraken_setpump setpump_msg;
    struct kraken_status status_msg;

    // 0=full speed, 1=manual override, 2=automatic
    int pump_enable;
    int fan_enable;
};

static int kraken_send_message(struct usb_kraken *kraken,
                               u8 *message, int length)
{
    int sent = 0;
    int retval = usb_bulk_msg(kraken->udev,
                              usb_sndintpipe(kraken->udev, 1),
                              message, length, &sent, 3000);
    if (retval != 0)
        return retval;
    if (sent != length)
        return -EIO;
    return 0;
}

static int kraken_receive_message(struct usb_kraken *kraken,
                                  u8 *message, int expected_length)
{
    int received = 0;
    int retval = usb_bulk_msg(kraken->udev,
                              usb_rcvintpipe(kraken->udev, 0x81),
                              message, expected_length, &received, 3000);
    if (retval != 0)
        return retval;
    if (received != expected_length)
        return -EIO;
    return 0;
}

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

            if (pt < min_pt)
                pt = min_pt;
            if (pt < 60)
                pt = 60;
            if (pt > 100)
                pt = 100;

            kraken->setpump_msg.pump_percent = pt;
        }

        if (kraken->fan_enable >= 2) {
            // Automatic fan speed

            // The fan throttle range
            // When liquid temp is min_lt, set fan throttle to 25%
            // When liquid temp is max_lt, set fan throttle to 100%
            const int min_ft = 66;
            const int max_ft = 100;
            const int rng_ft = max_ft - min_ft;

            int ft = ((ofs_lt * rng_ft) / rng_lt) + min_ft;

            if (ft < min_ft)
                ft = min_ft;
            if (ft > 100)
                ft = 100;

            kraken->setfan_msg.fan_percent = ft;
        }
    }

    retval = kraken_send_message(kraken, (u8*)&kraken->setfan_msg,
                                 sizeof(kraken->setfan_msg));
    if (retval < 0)
        goto send_failed;

    retval = kraken_receive_message(kraken, (u8*)&kraken->status_msg,
                                    sizeof(kraken->status_msg));
    if (retval < 0)
        goto receive_failed;

    retval = kraken_send_message(kraken, (u8*)&kraken->setpump_msg,
                                 sizeof(kraken->setpump_msg));
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

enum attr_index {
    // RW
    idx_pump_throttle,
    idx_fan_throttle,
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
    [idx_pump_throttle] = "pump_pwm",
    [idx_fan_throttle]  = "fan_pwm",
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
    [idx_pump_throttle] = { 0, 0xFF },
    [idx_fan_throttle]  = { 0, 0xFF },
    [idx_pump_enable]   = { 0, INT_MAX },
    [idx_fan_enable]    = { 0, INT_MAX }
};

static ssize_t attr_label_show(
        struct device *dev, struct device_attribute *attr, char *buf)
{
    const struct sensor_device_attribute *sensor_attr =
            to_sensor_dev_attr(attr);
    (void)dev;

    if (sensor_attr->index < idx_max)
        return sprintf(buf, "%s\n", attr_labels[sensor_attr->index]);

    return -EINVAL;
}

static u16 be16_bytes(const u8 *bytes)
{
    return (bytes[0] << 8) | bytes[1];
}

static ssize_t attr_show(
        struct device *dev, struct device_attribute *attr, char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);

    int value;

    switch (sensor_attr->index) {
    case idx_pump_throttle:
        value = 0xFF * kraken->setpump_msg.pump_percent / 100;
        break;

    case idx_fan_throttle:
        value = 0xFF * kraken->setfan_msg.fan_percent / 100;
        break;

    case idx_pump_enable:
        value = kraken->pump_enable;
        break;

    case idx_fan_enable:
        value = kraken->fan_enable;
        break;

    case idx_pump_rpm:
        value = be16_bytes(kraken->status_msg.pump_rpm);
        break;

    case idx_fan_rpm:
        value = be16_bytes(kraken->status_msg.fan_rpm);
        break;

    case idx_liquid_temp:
        value = kraken->status_msg.liquid_temp * 1000;
        break;

    default:
        return -EINVAL;
    }

    return sprintf(buf, "%d\n", value);
}

static ssize_t attr_store(
        struct device *dev, struct device_attribute *attr, const char *buf,
        size_t count)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);

    const struct attr_range *range;

    long value = 0;
    char *end = NULL;
    value = simple_strtol(buf, &end, 10);
    if (unlikely(!end))
        return -EINVAL;

    if (unlikely(sensor_attr->index < 0 || sensor_attr->index >=
                 (sizeof(attr_ranges) / sizeof(*attr_ranges))))
        return -EINVAL;

    range = &attr_ranges[sensor_attr->index];

    if (unlikely(value < range->min_value || value > range->max_value))
        return -EINVAL;

    switch (sensor_attr->index) {
    case idx_pump_throttle:
        kraken->setpump_msg.pump_percent = value * 100 / 0xFF;
        break;

    case idx_fan_throttle:
        kraken->setfan_msg.fan_percent = value * 100 / 0xFF;
        break;

    case idx_pump_enable:
        kraken->pump_enable = value;
        break;

    case idx_fan_enable:
        kraken->pump_enable = value;
        break;

    default:
        return -EINVAL;
    }

    return count;
}

static umode_t kraken_is_visible(
        struct kobject *kobj, struct attribute *attr, int index)
{
    return attr->mode;
}

// RW
static SENSOR_DEVICE_ATTR_RW(pwm1, attr, idx_pump_throttle);
static SENSOR_DEVICE_ATTR_RW(pwm2, attr, idx_fan_throttle);
static SENSOR_DEVICE_ATTR_RW(pwm1_enable, attr, idx_pump_enable);
static SENSOR_DEVICE_ATTR_RW(pwm2_enable, attr, idx_fan_enable);

// RO
static SENSOR_DEVICE_ATTR_RO(fan1_input, attr, idx_pump_rpm);
static SENSOR_DEVICE_ATTR_RO(fan2_input, attr, idx_fan_rpm);
static SENSOR_DEVICE_ATTR_RO(temp1_input, attr, idx_liquid_temp);

// RW labels
static SENSOR_DEVICE_ATTR_RO(pwm1_label, attr_label, idx_pump_throttle);
static SENSOR_DEVICE_ATTR_RO(pwm2_label, attr_label, idx_fan_throttle);
static SENSOR_DEVICE_ATTR_RO(pwm1_enable_label, attr_label, idx_pump_enable);
static SENSOR_DEVICE_ATTR_RO(pwm2_enable_label, attr_label, idx_fan_enable);

// RO labels
static SENSOR_DEVICE_ATTR_RO(fan1_label, attr_label, idx_pump_rpm);
static SENSOR_DEVICE_ATTR_RO(fan2_label, attr_label, idx_fan_rpm);
static SENSOR_DEVICE_ATTR_RO(temp1_label, attr_label, idx_liquid_temp);

static struct attribute *kraken_attrs[] = {
    // RW
    &sensor_dev_attr_pwm1.dev_attr.attr,
    &sensor_dev_attr_pwm2.dev_attr.attr,
    &sensor_dev_attr_pwm1_enable.dev_attr.attr,
    &sensor_dev_attr_pwm2_enable.dev_attr.attr,

    // RO
    &sensor_dev_attr_fan1_input.dev_attr.attr,
    &sensor_dev_attr_fan2_input.dev_attr.attr,
    &sensor_dev_attr_temp1_input.dev_attr.attr,

    // RW labels
    &sensor_dev_attr_pwm1_label.dev_attr.attr,
    &sensor_dev_attr_pwm2_label.dev_attr.attr,
    &sensor_dev_attr_pwm1_enable_label.dev_attr.attr,
    &sensor_dev_attr_pwm2_enable_label.dev_attr.attr,

    // RO labels
    &sensor_dev_attr_fan1_label.dev_attr.attr,
    &sensor_dev_attr_fan2_label.dev_attr.attr,
    &sensor_dev_attr_temp1_label.dev_attr.attr,

    NULL
};

static const struct attribute_group kraken_group = {
    .attrs = kraken_attrs,
    .is_visible = kraken_is_visible,
};
__ATTRIBUTE_GROUPS(kraken);

static void kraken_add_device_files(struct usb_interface *interface)
{
    struct usb_kraken *dev = usb_get_intfdata(interface);

    devm_hwmon_device_register_with_groups(
                &interface->dev, "kraken", dev, kraken_groups);
}

static enum hrtimer_restart update_timer_function(struct hrtimer *update_timer)
{
    struct usb_kraken *dev = container_of(update_timer,
                                          struct usb_kraken, update_timer);
    queue_work(dev->update_workqueue, &dev->update_work);
    hrtimer_forward(update_timer, ktime_get(), ktime_set(1, 0));
    return HRTIMER_RESTART;
}

static void update_work_function(struct work_struct *param)
{
    struct usb_kraken *dev = container_of(
                param, struct usb_kraken, update_work);
    kraken_update(dev);
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

    pr_info("Probing for NZXT X52 water cooler\n");

    dev = devm_kzalloc(&interface->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        goto error_dev;

    // Default to automatic
    dev->pump_enable = 2;
    dev->fan_enable = 2;

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

    // Initialize polling timer
    hrtimer_init(&dev->update_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

    dev->update_timer.function = &update_timer_function;
    hrtimer_start(&dev->update_timer, ktime_set(1, 0), HRTIMER_MODE_REL);

    dev->update_workqueue = create_singlethread_workqueue("kraken_up");
    INIT_WORK(&dev->update_work, update_work_function);

    // Initialize hwmon device
    kraken_add_device_files(interface);

    dev_info(&interface->dev, "Kraken connected\n");
    return 0;

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

    flush_workqueue(dev->update_workqueue);
    destroy_workqueue(dev->update_workqueue);
    hrtimer_cancel(&dev->update_timer);

    dev_info(&interface->dev, "Kraken disconnected\n");
}

static struct usb_driver kraken_driver = {
    .name =		  "kraken",
    .probe =	  kraken_probe,
    .disconnect = kraken_disconnect,
    .id_table =	  id_table,
};

module_usb_driver(kraken_driver);

MODULE_SUPPORTED_DEVICE("Kraken X52 240mm liquid cooler");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("NZXT Kraken X52 driver");
MODULE_AUTHOR("Doug Gale <doug16k@gmail.com>");
MODULE_LICENSE("GPL");
