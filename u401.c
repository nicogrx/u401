/*
This file is part of the U401 library.

The U401 library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The U401 library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with The U401 library.  If not, see <http://www.gnu.org/licenses/>.

Author: Jean-Nicolas Graux <nicogrx@gmail.com>
Revision: 1.0

This provides very basic support for the U401 usb board from
http://www.usbmicro.com/.

Only a subset of U401 functionalities is supported up to now.
Basically, the functions to control PORTA, PORTB gpios.
*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <usb.h>
#include "u401.h"

#ifdef DEBUG
#define PDEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define PDEBUG(fmt, ...)
#endif

#define CANT_SEND -1
#define CANT_READ -2

#define U401_VENDOR_ID  0x0DE7
#define U401_PRODUCT_ID 0x0191

enum U401_CMDS
{
	U401_CMD_INITPORT = 0,
	U401_CMD_WRITEA,
	U401_CMD_WRITEB,
	U401_CMD_WRITEABIT,
	U401_CMD_WRITEBBIT,
	U401_CMD_READA,
	U401_CMD_READB,
	U401_CMD_SETBIT,
	U401_CMD_RESETBIT,
	U401_CMD_DIRA,
	U401_CMD_DIRB,
};

enum U401_GPIO_PORTS
{
	PORTA = 0,
	PORTB,
};

struct u401_irq {
	void (*isr)(int);
	int trigger;
	bool enabled;
};

static pthread_t isr_thread;
static pthread_mutex_t u401_lock;
static char porta_saved_dir0 = 0;
static char portb_saved_dir0 = 0;
static char porta_saved_dir1 = 0;
static char portb_saved_dir1 = 0;
static bool end = false;
static struct u401_irq u401_irqs[U401_GPIOS];
static struct usb_dev_handle *u401_handle = NULL;
static bool interrupt_handler_initialized = false;

/* This function sends the data contained in the buffer directly to the device.	*/
/* Taken from USBMicro's documentation for Linux interfaces.			*/
static int u401_send_command(char *command, int comLen, int resLen )
{
        int ret;
	pthread_mutex_lock(&u401_lock);
	ret = usb_control_msg(u401_handle, 0x21, 9, 0x0200, 0, command, comLen, 5000);
        if (ret != comLen) {
		pthread_mutex_unlock(&u401_lock);
		return CANT_SEND;
	}
        if (resLen > 0) {
                ret = usb_bulk_read(u401_handle, 0x81, command, resLen, 5000);
		pthread_mutex_unlock(&u401_lock);
                if (ret != resLen) return CANT_READ;
        }
        return ret;
}

static struct usb_device *find_usb_device(uint16_t vendor, uint16_t product)
{
	struct usb_bus *bus;
	struct usb_device *dev;
	struct usb_bus *busses;
	usb_init();
	usb_find_busses();
	usb_find_devices();
	busses = usb_get_busses();
	if (!busses) {
		return NULL;
		PDEBUG("%s: failed to get usb busses\n", __func__);
	}
	for (bus = busses; bus; bus = bus->next)
		for (dev = bus->devices; dev; dev = dev->next)
			if ((dev->descriptor.idVendor == vendor) &&
				(dev->descriptor.idProduct == product))
				return dev;
	return NULL;

}

static int u401_read_port(int port, char *data)
{
	int ret = 0;
	char buf[8];
	memset(buf, 0, sizeof(buf));
	buf[0] = (port == PORTA ? U401_CMD_READA : U401_CMD_READB);
	ret = u401_send_command(buf, sizeof(buf), sizeof(buf));
	if (ret != sizeof(buf))
	{
		PDEBUG("cannot read port%s\n", port == PORTA ? "A" : "B");
		return -ENODEV;
	}
	*data = buf[1];
	return 0;

}

static int u401_write_port(int port, char data)
{
	int ret = 0;
	char buf[8];
	memset(buf, 0, sizeof(buf));
	buf[0] = (port == PORTA ? U401_CMD_WRITEA : U401_CMD_WRITEB);
	buf[1] = data;
	ret = u401_send_command(buf, sizeof(buf), sizeof(buf));
	if (ret != sizeof(buf))
	{
		PDEBUG("cannot write port%s\n", port == PORTA ? "A" : "B");
		return -1;
	}
	return 0;
}

static int u401_write_port_bits(int port, char bits_to_set, char bits_to_clear)
{
	int ret = 0;
	char buf[8];
	memset(buf, 0, sizeof(buf));
	buf[0] = (port == PORTA ? U401_CMD_WRITEABIT : U401_CMD_WRITEBBIT);
	buf[1] = ~bits_to_clear;
	buf[2] = bits_to_set;

	PDEBUG("%s: port=%x, bits_to_set=%x, bits_to_clear=%x, %x,%x,%x,%x,%x,%x,%x,%x\n",
		__func__, port, bits_to_set, bits_to_clear,
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7]);


	ret = u401_send_command(buf, sizeof(buf), sizeof(buf));
	if (ret != sizeof(buf))
	{
		PDEBUG("cannot write port%s bits\n", port == PORTA ? "A" : "B");
		return -1;
	}
	return 0;
}

static int u401_set_port(int port, char dir0, char dir1)
{
	int ret = 0;
	char buf[8];
	memset(buf, 0, sizeof(buf));
	buf[0] = (port == PORTA ? U401_CMD_DIRA : U401_CMD_DIRB);
	buf[1] = dir0;
	buf[2] = dir1;

	PDEBUG("%s: port=%x, dir0=%x, dir1=%x, %x,%x,%x,%x,%x,%x,%x,%x\n",
		__func__, port, dir0, dir1,
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7]);

	ret = u401_send_command(buf, sizeof(buf), sizeof(buf));
	if (ret != sizeof(buf))
	{
		PDEBUG("cannot set port%s\n", port == PORTA ? "A" : "B");
		return -1;
	}
	return 0;
}

static int u401_gpio_to_port(int id, int *port, char *msk)
{
	if (id >= U401_GPIOS)
		return -ENODEV;
	if (id > 7) {
		*port = PORTB;
		*msk = (char)((1 << id) >> 8);
	} else {
		*port = PORTA;
		*msk = (char)(1 << id);
	}
	return 0;
}

static int u401_reset_ports(void)
{
	int ret = 0;
	char buf[8];
	memset(buf, 0, sizeof(buf));
	buf[0] = U401_CMD_INITPORT;
	ret = u401_send_command(buf, sizeof(buf), sizeof(buf));
	if (ret != sizeof(buf))
	{
		PDEBUG("cannot reset ports\n");
		return -1;
	}
	return 0;
}

static void *gpio_interrupt_handler(void *ptr)
{
	int i, ret;
	struct u401_irq *ui;
	char porta, portb;
	int prev_gpio_values, gpio_values;

	while (!end) {
		usleep(50);
		ret = u401_read_port(PORTA, &porta);
		if (ret)
			continue;
		ret = u401_read_port(PORTB, &portb);
		if (ret)
			continue;
		gpio_values = porta | (portb << 8);

		for (i = 0; i < U401_GPIOS; i++) {
			ui = &u401_irqs[i];
			if (!ui->enabled)
				continue;
			if (!ui->isr) {
				PDEBUG("%s: irq%i is enabled but isr is NULL\n",
					__func__, i);
				continue;
			}
			if (ui->trigger == IRQ_RISING_FALLING) {
				if ((gpio_values & (1 << i)) != (prev_gpio_values & (1 << i))) {
					ui->isr(i);
				}
			} else if (ui->trigger == IRQ_RISING) {
				if ((gpio_values & (1 << i)) && (!(prev_gpio_values & (1 << i)))) {
					ui->isr(i);
				}
			} else if (ui->trigger == IRQ_FALLING) {
				if (!((gpio_values & (1 << i))) && (prev_gpio_values & (1 << i))) {
					ui->isr(i);
				}
			}
		}
		prev_gpio_values = gpio_values;
	}
}

int u401_attach(void)
{
	int ret = 0;
	int saver_status;
	struct usb_device *dev;

	dev = find_usb_device(U401_VENDOR_ID,U401_PRODUCT_ID);
	if (!dev)
		return -ENODEV;

	PDEBUG("%s: found u401: %s %04X/%04X\n",
		__func__,
		dev->filename,
		dev->descriptor.idVendor,
		dev->descriptor.idProduct);

	u401_handle = usb_open(dev);
	if (!dev->config)
	{
		PDEBUG("%s: couldn't retrieve descriptors\n", __func__);
		return -ENODEV;
	}

	usb_detach_kernel_driver_np(u401_handle, 0);
	usb_set_altinterface(u401_handle, 0);
	if(usb_claim_interface(u401_handle, 0))
	{
		PDEBUG("%s: failed to claim the USB device\n", __func__);
		usb_close(u401_handle);
		return -ENODEV;
	}

	if (pthread_mutex_init(&u401_lock, NULL) != 0) {
		PDEBUG("%s: mutex init failed\n", __func__);
		usb_close(u401_handle);
		return -ENODEV;
	}

	if (u401_reset_ports()) {
		usb_close(u401_handle);
		return -ENODEV;
	}
	PDEBUG("%s: done\n", __func__);
	return 0;
}

void u401_detach(void)
{
	end = true;
	pthread_join (isr_thread, NULL);
	usb_release_interface(u401_handle, 0);
	usb_close(u401_handle);
	interrupt_handler_initialized = false;
}

int u401_gpio_set_value(int id, bool value)
{
	int port, ret = 0;
	char msk, data;
	if (ret = u401_gpio_to_port(id, &port, &msk))
		return ret;
	if (value)
		ret = u401_write_port_bits(port, msk, 0);
	else
		ret = u401_write_port_bits(port, 0, msk);
	return ret;
}

int u401_gpio_get_value(int id, int *value)
{
	int port, ret = 0;
	char msk, data;
	if (ret = u401_gpio_to_port(id, &port, &msk))
		return ret;
	ret = u401_read_port(port, &data);

	PDEBUG("%s: port=%i, data=%x\n", __func__, port, data);

	if (!ret)
		*value = !!(msk & data);
	return ret;
}

int u401_gpio_setup(int id, bool dir)
{
	int port, ret = 0;
	char msk, data;
	char *saved_dir0;
	char *saved_dir1;

	if (ret = u401_gpio_to_port(id, &port, &msk))
		return ret;
	pthread_mutex_lock(&u401_lock);
	saved_dir0 = (port == PORTB ? &portb_saved_dir0 : &porta_saved_dir0);
	saved_dir1 = (port == PORTB ? &portb_saved_dir1 : &porta_saved_dir1);
	if (dir) {
		*saved_dir0 |= msk;
		*saved_dir1 |= msk;
	} else {
		*saved_dir0 &= ~msk;
		*saved_dir1 &= ~msk;
	}
	pthread_mutex_unlock(&u401_lock);
	ret = u401_set_port(port, *saved_dir0, *saved_dir1);
	return ret;
}

int u401_gpio_request_irq(int id, int trigger, void (*isr)(int))
{
	int ret, val;
	struct u401_irq *ui;

	if (!u401_handle)
		return -ENODEV;
	if (id < 0 || id >= U401_GPIOS)
		return -ENODEV;
	if (isr == NULL)
		return -EFAULT;

	pthread_mutex_lock(&u401_lock);

	if (!interrupt_handler_initialized) {
		memset(u401_irqs, 0, U401_GPIOS * sizeof(struct u401_irq));
		ret = pthread_create(&isr_thread, NULL, gpio_interrupt_handler, NULL);
		if(ret) {
			PDEBUG("%s: failed to create thread: err=%d\n", __func__, ret);
			pthread_mutex_unlock(&u401_lock);
			return ret;
		}
		interrupt_handler_initialized = true;
	}

	ui = &u401_irqs[id];

	if (ui->isr != NULL) {
		PDEBUG("%s: irq with id=%i is already requested\n", __func__, id);
		pthread_mutex_unlock(&u401_lock);
		return -EFAULT;
	}

	ui->isr = isr;
	ui->trigger = trigger;
	ui->enabled = true;

	pthread_mutex_unlock(&u401_lock);
	return 0;
}
