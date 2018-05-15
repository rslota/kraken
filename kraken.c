#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>

// NZXT Kraken X52 120mm Liquid Cooler
#define VENDOR_ID   0x1E71
#define PRODUCT_ID  0x170E

#define MIN_THROTTLE    50
#define MAX_THROTTLE    100

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
    u8 auto_throttle;
};

static int kraken_probe(struct usb_interface *interface,
                        const struct usb_device_id *id);
static void kraken_disconnect(struct usb_interface *interface);

static struct usb_driver kraken_driver = {
    .name =		"kraken",
    .probe =	kraken_probe,
    .disconnect = kraken_disconnect,
    .id_table =	id_table,
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

    if (kraken->auto_throttle) {
        // When liquid temp is 30, set fan and pump throttle to 65%
        // When liquid temp is 39, set fan and pump throttle to 85%

        int level = (((kraken->status_msg.liquid_temp - 30) * 20) / 9) + 65;

        if (level < 65)
            level = 65;
        if (level > 100)
            level = 100;

        kraken->setfan_msg.fan_percent = level;
        kraken->setpump_msg.pump_percent = level;
    }

    retval = kraken_send_message(kraken, (u8*)&kraken->setfan_msg,
                                 sizeof(kraken->setfan_msg));
    if (retval < 0)
        goto send_failed;

    retval = kraken_receive_message(kraken, (u8*)&kraken->status_msg,
                                    sizeof(kraken->status_msg));
    if (retval < 0)
        goto received_failed;

    retval = kraken_send_message(kraken, (u8*)&kraken->setpump_msg,
                                 sizeof(kraken->setpump_msg));
    if (retval < 0)
        goto send_failed;

    retval = kraken_receive_message(kraken, (u8*)&kraken->status_msg,
                                    sizeof(kraken->status_msg));
    if (retval < 0)
        goto received_failed;

    return 0;

send_failed:
    dev_err(&kraken->udev->dev, "Failed to send update: %d\n", retval);
    return retval;

received_failed:
    dev_err(&kraken->udev->dev, "Failed to receive: %d\n", retval);
    return retval;
}

//
// Pump throttle

static ssize_t show_pump_throttle(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    return sprintf(buf, "%u\n", kraken->setpump_msg.pump_percent);
}

static ssize_t set_pump_throttle(struct device *dev,
                         struct device_attribute *attr, const char *buf,
                         size_t count)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    int speed = 0;
    if (sscanf(buf, "%d", &speed) != 1 ||
            speed < MIN_THROTTLE || speed > MAX_THROTTLE)
        return -EINVAL;

    kraken->setpump_msg.pump_percent = speed;

    return count;
}

static DEVICE_ATTR(pump_throttle, S_IRUGO | S_IWUSR | S_IWGRP,
                   show_pump_throttle, set_pump_throttle);

//
// Fan throttle

static ssize_t show_fan_throttle(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    return sprintf(buf, "%u\n", kraken->setfan_msg.fan_percent);
}

static ssize_t set_fan_throttle(struct device *dev,
                         struct device_attribute *attr, const char *buf,
                         size_t count)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    int speed;
    if (sscanf(buf, "%d", &speed) != 1 ||
            speed < MIN_THROTTLE || speed > MAX_THROTTLE)
        return -EINVAL;

    kraken->setfan_msg.fan_percent = speed;

    return count;
}

static DEVICE_ATTR(fan_throttle, S_IRUGO | S_IWUSR | S_IWGRP,
                   show_fan_throttle, set_fan_throttle);

//
// Auto throttle

static ssize_t show_auto_throttle(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    return sprintf(buf, "%u\n", kraken->auto_throttle);
}

static ssize_t set_auto_throttle(struct device *dev,
                         struct device_attribute *attr, const char *buf,
                         size_t count)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    int auto_throttle;
    if (sscanf(buf, "%d", &auto_throttle) != 1 ||
            auto_throttle < 0 || auto_throttle > 1)
        return -EINVAL;

    kraken->auto_throttle = (auto_throttle != 0);

    return count;
}

static DEVICE_ATTR(auto_throttle, S_IRUGO | S_IWUSR | S_IWGRP,
                   show_auto_throttle, set_auto_throttle);

//
// Liquid temp

static ssize_t show_liquid_temp(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    return sprintf(buf, "%u\n", kraken->status_msg.liquid_temp);
}

static DEVICE_ATTR(liquid_temp, S_IRUGO, show_liquid_temp, NULL);

//
// Pump rpm

static ssize_t show_pump_rpm(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    unsigned pump_rpm = (kraken->status_msg.pump_rpm[0] << 8) |
            kraken->status_msg.pump_rpm[1];

    return sprintf(buf, "%u\n", pump_rpm);
}

static DEVICE_ATTR(pump_rpm, S_IRUGO, show_pump_rpm, NULL);

//
// Fan rpm

static ssize_t show_fan_rpm(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
    struct usb_interface *intf = to_usb_interface(dev);
    struct usb_kraken *kraken = usb_get_intfdata(intf);

    unsigned fan_rpm = (kraken->status_msg.fan_rpm[0] << 8) |
            kraken->status_msg.fan_rpm[1];

    return sprintf(buf, "%u\n", fan_rpm);
}

static DEVICE_ATTR(fan_rpm, S_IRUGO, show_fan_rpm, NULL);

static void kraken_remove_device_files(struct usb_interface *interface)
{
    device_remove_file(&interface->dev, &dev_attr_pump_throttle);
    device_remove_file(&interface->dev, &dev_attr_fan_throttle);
    device_remove_file(&interface->dev, &dev_attr_liquid_temp);
    device_remove_file(&interface->dev, &dev_attr_pump_rpm);
    device_remove_file(&interface->dev, &dev_attr_fan_rpm);
    device_remove_file(&interface->dev, &dev_attr_auto_throttle);
}

enum hrtimer_restart update_timer_function(struct hrtimer *update_timer)
{
    struct usb_kraken *dev = container_of(update_timer,
                                          struct usb_kraken, update_timer);
    queue_work(dev->update_workqueue, &dev->update_work);
    hrtimer_forward(update_timer, ktime_get(), ktime_set(1, 0));
    return HRTIMER_RESTART;
}

static void update_work_function(struct work_struct *param)
{
    struct usb_kraken *dev = container_of(param,
                                          struct usb_kraken, update_work);
    kraken_update(dev);
}

static int kraken_probe(struct usb_interface *interface,
                        const struct usb_device_id *id)
{
    struct usb_device *udev;
    struct usb_kraken *dev;
    int retval;

    udev = interface_to_usbdev(interface);
    dev = NULL;
    retval = -ENOMEM;

    pr_info("Probing for NZXT X52 water cooler\n");

    dev = kmalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev)
        goto error_dev;

    memset(dev, 0, sizeof(*dev));

    dev->auto_throttle = 1;

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

    dev_info(&interface->dev, "Kraken creating device files\n");

    if ((retval = device_create_file(&interface->dev,
                                     &dev_attr_pump_throttle)) ||
        (retval = device_create_file(&interface->dev,
                                     &dev_attr_fan_throttle)) ||
        (retval = device_create_file(&interface->dev,
                                     &dev_attr_liquid_temp)) ||
        (retval = device_create_file(&interface->dev,
                                     &dev_attr_pump_rpm)) ||
        (retval = device_create_file(&interface->dev,
                                     &dev_attr_fan_rpm)) ||
        (retval = device_create_file(&interface->dev,
                                     &dev_attr_auto_throttle))) {
        dev_err(&interface->dev, "Error creating device files\n");
        goto error;
    }

    u8 *descriptor = kmalloc(130, GFP_KERNEL);

    retval = usb_control_msg(udev,
                             usb_sndctrlpipe(udev, 0),
                             6, 0x80, 0x0303, 0x409,
                             descriptor, sizeof(descriptor), 1000);

    kfree(descriptor);

    if (retval < 0) {
        dev_err(&interface->dev, "Error sending initial control message: %d\n",
                retval);
        goto error;
    }

    dev_info(&interface->dev, "Kraken connected\n");
    hrtimer_init(&dev->update_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    dev->update_timer.function = &update_timer_function;
    hrtimer_start(&dev->update_timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    dev->update_workqueue = create_singlethread_workqueue("kraken_up");
    INIT_WORK(&dev->update_work, update_work_function);
    return 0;

error:
    kraken_remove_device_files(interface);

    usb_set_intfdata(interface, NULL);
    usb_put_dev(dev->udev);
    kfree(dev);

error_dev:
    return retval;
}

static void kraken_disconnect(struct usb_interface *interface)
{
    struct usb_kraken *dev = usb_get_intfdata(interface);

    kraken_remove_device_files(interface);

    usb_set_intfdata(interface, NULL);
    usb_put_dev(dev->udev);

    flush_workqueue(dev->update_workqueue);
    destroy_workqueue(dev->update_workqueue);
    hrtimer_cancel(&dev->update_timer);
    kfree(dev);

    dev_info(&interface->dev, "Kraken disconnected\n");
}

module_usb_driver(kraken_driver);

MODULE_LICENSE("GPL");
