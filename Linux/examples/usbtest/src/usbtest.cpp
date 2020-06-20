//============================================================================
// Name        : usbtest.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright 2018, I-SYST
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/utsname.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <memory>
#include <chrono>
#include <string>
#include <map>
#include <list>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>

/* Linux */
#include <linux/hidraw.h>
#include <linux/version.h>
#include <linux/input.h>
#include <libudev.h>

//#include <sd-device.h>

pthread_t g_ProgThreadHandle[20];
volatile int g_ThreadCnt = 0;
volatile int g_ThreadReady = 0;
std::list<pthread_t> g_ThreadList;
sem_t g_ThreadSem;
pthread_mutex_t g_ProgStartMutex;
pthread_cond_t g_ProgStartCond;

static uint32_t detect_kernel_version(void)
{
	struct utsname name;
	int major, minor, release;
	int ret;

	uname(&name);
	ret = sscanf(name.release, "%d.%d.%d", &major, &minor, &release);
	if (ret == 3) {
		return KERNEL_VERSION(major, minor, release);
	}

	ret = sscanf(name.release, "%d.%d", &major, &minor);
	if (ret == 2) {
		return KERNEL_VERSION(major, minor, 0);
	}

	printf("Couldn't determine kernel version from version string \"%s\"\n", name.release);
	return 0;
}

/*
 * The caller is responsible for free()ing the (newly-allocated) character
 * strings pointed to by serial_number_utf8 and product_name_utf8 after use.
 */
bool GetId(struct udev_device *pDev, uint16_t &Vid, uint16_t &Pid)
{
	uint16_t bus;

	struct udev_device * parent =
		udev_device_get_parent_with_subsystem_devtype(pDev, "hid", NULL);
	const char * attval = udev_device_get_sysattr_value(parent, "uevent");

	char *s = strtok((char*)attval, "\n");

	while (s != NULL ) {
		if (strncmp("HID_ID", s, 6) == 0)
		{
			sscanf(&s[7], "%x:%hx:%hx", &bus, &Vid, &Pid);
			break;
		}
		s = strtok(NULL, "\n");
	};

	return true;
}


int main() {
	uint32_t kvers = detect_kernel_version();

	struct udev *udev;
	struct udev_enumerate *enumerate;
//	struct udev_list_entry *device, *dev_list_entry;

	struct hid_device_info *root = NULL; /* return object */
	struct hid_device_info *cur_dev = NULL;
	struct hid_device_info *prev_dev = NULL; /* previous device */

	/* Create the udev object */
	udev = udev_new();
	if (!udev) {
		printf("Can't create udev\n");
		return 0;
	}

    pthread_mutex_init(&g_ProgStartMutex, NULL);
	pthread_cond_init(&g_ProgStartCond, NULL);

	/* Create a list of the devices in the 'hidraw' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "hidraw");
	udev_enumerate_scan_devices(enumerate);

	struct udev_list_entry *item = udev_enumerate_get_list_entry(enumerate);

	while (item != NULL)
	{
		const char * devpath = udev_list_entry_get_name(item);

		printf(devpath);
		printf("\n");

		struct udev_device *raw_dev = udev_device_new_from_syspath(udev, devpath);

		uint16_t vid = 0;
		uint16_t pid = 0;

		GetId(raw_dev, vid, pid);

		printf("Vend ID = %x, Prod ID = %x\n", vid, pid);


		item = udev_list_entry_get_next(item);
	}
#if 0
	{
			const char *sysfs_path;
			const char *dev_path;
			const char *str;
			struct udev_device *raw_dev; /* The device's hidraw udev node. */
			struct udev_device *hid_dev; /* The device's HID udev node. */
			struct udev_device *usb_dev; /* The device's USB udev node. */
			struct udev_device *intf_dev; /* The device's interface (in the USB sense). */
			unsigned short dev_vid;
			unsigned short dev_pid;
			char *serial_number_utf8 = NULL;
			char *product_name_utf8 = NULL;
			int bus_type;
			int result;

			/* Get the filename of the /sys entry for the device
			   and create a udev_device object (dev) representing it */
			sysfs_path = udev_list_entry_get_name(dev_list_entry);
			raw_dev = udev_device_new_from_syspath(udev, sysfs_path);
			dev_path = udev_device_get_devnode(raw_dev);

			hid_dev = udev_device_get_parent_with_subsystem_devtype(
				raw_dev,
				"hid",
				NULL);

			if (!hid_dev) {
				/* Unable to find parent hid device. */
				goto next;
			}

			result = parse_uevent_info(
				udev_device_get_sysattr_value(hid_dev, "uevent"),
				&bus_type,
				&dev_vid,
				&dev_pid,
				&serial_number_utf8,
				&product_name_utf8);

			if (!result) {
				/* parse_uevent_info() failed for at least one field. */
				goto next;
			}

			if (bus_type != BUS_USB && bus_type != BUS_BLUETOOTH) {
				/* We only know how to handle USB and BT devices. */
				goto next;
			}


		next:
			free(serial_number_utf8);
			free(product_name_utf8);
			udev_device_unref(raw_dev);
			/* hid_dev, usb_dev and intf_dev don't need to be (and can't be)
			   unref()d.  It will cause a double-free() error.  I'm not
			   sure why.  */

		/* Free the enumerator and udev objects. */
		udev_enumerate_unref(enumerate);
		udev_unref(udev);
	}
#endif
	udev_enumerate_unref(enumerate);
	udev_unref(udev);

	printf("Hello %x\n", kvers);
	return 0;
}
