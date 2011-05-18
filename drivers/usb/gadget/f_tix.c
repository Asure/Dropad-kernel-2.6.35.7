/*
 * Gadget driver for Ergotest TiX
 * 
 * Copyright (C) 2011 Ergotest Innovation as
 * Author: Karel Boek <karel.boek@ergotest.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed  WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE.  See the GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/usb/android_composite.h>

#define BULK_BUFFER_SIZE           4096

/* number of tx requests to allocate */
#define TX_REQ_MAX 4

static const char shortname[] = "android_tix";

struct tix_dev {
	struct usb_function 		function;
	struct usb_composite_dev 	*cdev;
	
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	int online;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;

	struct list_head tx_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	
	struct usb_request *rx_req;
	
	int rx_done;
};

static struct usb_interface_descriptor tix_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0x42,
	.bInterfaceProtocol     = 1,
};

static struct usb_endpoint_descriptor tix_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor tix_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor tix_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor tix_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_tix_descs[] = {
	(struct usb_descriptor_header *) &tix_interface_desc,
	(struct usb_descriptor_header *) &tix_fullspeed_in_desc,
	(struct usb_descriptor_header *) &tix_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_tix_descs[] = {
	(struct usb_descriptor_header *) &tix_interface_desc,
	(struct usb_descriptor_header *) &tix_highspeed_in_desc,
	(struct usb_descriptor_header *) &tix_highspeed_out_desc,
	NULL,
};

/* temporary variable used between tix_open() and tix_gadget_bind() */
static struct tix_dev *_tix_dev;

static atomic_t tix_enable_excl;

static inline struct tix_dev *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct tix_dev, function);
}

static struct usb_request *tix_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void tix_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
void request_put(struct tix_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
struct usb_request *request_get(struct tix_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void tix_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct tix_dev *dev = _tix_dev;

	if (req->status != 0)
		dev->error = 1;

	request_put(dev, &dev->tx_idle, req);

	wake_up(&dev->write_wq);
}

static void tix_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct tix_dev *dev = _tix_dev;

	dev->rx_done = 1;
	if (req->status != 0)
		dev->error = 1;

	wake_up(&dev->read_wq);
}

static int __init create_bulk_endpoints(struct tix_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for tix ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	/* now allocate requests for our endpoints */
	req = tix_request_new(dev->ep_out, BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = tix_complete_out;
	dev->rx_req = req;

	for (i = 0; i < TX_REQ_MAX; i++) {
		req = tix_request_new(dev->ep_in, BULK_BUFFER_SIZE);
		if (!req)
			goto fail;
		req->complete = tix_complete_in;
		request_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	printk(KERN_ERR "tix_bind() could not allocate requests\n");
	return -1;
}

static ssize_t tix_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	struct tix_dev *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	int r = count, xfer;
	int ret;

	DBG(cdev, "tix_read(%d)\n", count);

	if (count > BULK_BUFFER_SIZE)
		return -EINVAL;

	if (_lock(&dev->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(dev->online || dev->error)) {
		DBG(cdev, "tix_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->read_wq,
				(dev->online || dev->error));
		if (ret < 0) {
			_unlock(&dev->read_excl);
			return ret;
		}
	}
	if (dev->error) {
		r = -EIO;
		goto done;
	}

requeue_req:
	/* queue a request */
	req = dev->rx_req;
	req->length = count;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
	if (ret < 0) {
		DBG(cdev, "tix_read: failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		dev->error = 1;
		goto done;
	} else {
		DBG(cdev, "rx %p queue\n", req);
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible(dev->read_wq, dev->rx_done);
	if (ret < 0) {
		dev->error = 1;
		r = ret;
		goto done;
	}
	if (!dev->error) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0)
			goto requeue_req;

		DBG(cdev, "rx %p %d\n", req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;
		if (copy_to_user(buf, req->buf, xfer))
			r = -EFAULT;
	} else
		r = -EIO;

done:
	_unlock(&dev->read_excl);
	DBG(cdev, "tix_read returning %d\n", r);
	return r;
}

static ssize_t tix_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct tix_dev *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG(cdev, "tix_write(%d)\n", count);

	if (_lock(&dev->write_excl))
		return -EBUSY;

	while (count > 0) {
		if (dev->error) {
			DBG(cdev, "tix_write dev->error\n");
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
			((req = request_get(dev, &dev->tx_idle)) || dev->error));

		if (ret < 0) {
			r = ret;
			break;
		}

		if (req != 0) {
			if (count > BULK_BUFFER_SIZE)
				xfer = BULK_BUFFER_SIZE;
			else
				xfer = count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}

			req->length = xfer;
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) {
				DBG(cdev, "tix_write: xfer error %d\n", ret);
				dev->error = 1;
				r = -EIO;
				break;
			}

			buf += xfer;
			count -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}

	if (req)
		request_put(dev, &dev->tx_idle, req);

	_unlock(&dev->write_excl);
	DBG(cdev, "tix_write returning %d\n", r);
	return r;
}

static int tix_open(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "tix_open\n");
	if (_lock(&_tix_dev->open_excl))
		return -EBUSY;

	fp->private_data = _tix_dev;

	/* clear the error latch */
	_tix_dev->error = 0;

	return 0;
}

static int tix_release(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "tix_release\n");
	_unlock(&_tix_dev->open_excl);
	return 0;
}

/* file operations for tix device /dev/android_tix */
static struct file_operations tix_fops = {
	.owner = THIS_MODULE,
	.read = tix_read,
	.write = tix_write,
	.open = tix_open,
	.release = tix_release,
};

static struct miscdevice tix_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = shortname,
	.fops = &tix_fops,
};

static int tix_enable_open(struct inode *ip, struct file *fp)
{
	if (atomic_inc_return(&tix_enable_excl) != 1) {
		atomic_dec(&tix_enable_excl);
		return -EBUSY;
	}

	printk(KERN_INFO "enabling tix\n");
	android_enable_function(&_tix_dev->function, 1);

	return 0;
}

static int tix_enable_release(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "disabling tix\n");
	android_enable_function(&_tix_dev->function, 0);
	atomic_dec(&tix_enable_excl);
	return 0;
}

static const struct file_operations tix_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    tix_enable_open,
	.release = tix_enable_release,
};

static struct miscdevice tix_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_tix_enable",
	.fops = &tix_enable_fops,
};

static int
tix_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct tix_dev	*dev = func_to_dev(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
	DBG(cdev, "tix_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	tix_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = create_bulk_endpoints(dev, &tix_fullspeed_in_desc,
			&tix_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		tix_highspeed_in_desc.bEndpointAddress =
			tix_fullspeed_in_desc.bEndpointAddress;
		tix_highspeed_out_desc.bEndpointAddress =
			tix_fullspeed_out_desc.bEndpointAddress;
	}

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
tix_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct tix_dev	*dev = func_to_dev(f);
	struct usb_request *req;

	spin_lock_irq(&dev->lock);

	tix_request_free(dev->rx_req, dev->ep_out);
	while ((req = request_get(dev, &dev->tx_idle)))
		tix_request_free(req, dev->ep_in);

	dev->online = 0;
	dev->error = 1;
	spin_unlock_irq(&dev->lock);

	misc_deregister(&tix_device);
	misc_deregister(&tix_enable_device);
	kfree(_tix_dev);
	_tix_dev = NULL;
}

static int tix_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct tix_dev	*dev = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	DBG(cdev, "tix_function_set_alt intf: %d alt: %d\n", intf, alt);
	ret = usb_ep_enable(dev->ep_in,
			ep_choose(cdev->gadget,
				&tix_highspeed_in_desc,
				&tix_fullspeed_in_desc));
	if (ret)
		return ret;
	ret = usb_ep_enable(dev->ep_out,
			ep_choose(cdev->gadget,
				&tix_highspeed_out_desc,
				&tix_fullspeed_out_desc));
	if (ret) {
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	dev->online = 1;

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}

static void tix_function_disable(struct usb_function *f)
{
	struct tix_dev	*dev = func_to_dev(f);
	struct usb_composite_dev	*cdev = dev->cdev;

	DBG(cdev, "tix_function_disable\n");
	dev->online = 0;
	dev->error = 1;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static int tix_bind_config(struct usb_configuration *c)
{
	struct tix_dev *dev;
	int ret;

	printk(KERN_INFO "tix_bind_config\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spin_lock_init(&dev->lock);

	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);

	INIT_LIST_HEAD(&dev->tx_idle);

	dev->cdev = c->cdev;
	dev->function.name = "tix";
	dev->function.descriptors = fs_tix_descs;
	dev->function.hs_descriptors = hs_tix_descs;
	dev->function.bind = tix_function_bind;
	dev->function.unbind = tix_function_unbind;
	dev->function.set_alt = tix_function_set_alt;
	dev->function.disable = tix_function_disable;

	/* start disabled */
	dev->function.disabled = 1;

	/* _tix_dev must be set before calling usb_gadget_register_driver */
	_tix_dev = dev;

	ret = misc_register(&tix_device);
	if (ret)
		goto err1;
	ret = misc_register(&tix_enable_device);
	if (ret)
		goto err2;

	ret = usb_add_function(c, &dev->function);
	if (ret)
		goto err3;

	return 0;

err3:
	misc_deregister(&tix_enable_device);
err2:
	misc_deregister(&tix_device);
err1:
	kfree(dev);
	printk(KERN_ERR "tix gadget driver failed to initialize\n");
	return ret;
}

static struct android_usb_function tix_function = {
	.name = "tix",
	.bind_config = tix_bind_config,
};

static int __init init(void)
{
	printk(KERN_INFO "f_tix init\n");
	android_register_function(&tix_function);
	return 0;
}
module_init(init);
