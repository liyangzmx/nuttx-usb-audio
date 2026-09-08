/****************************************************************************
 * drivers/usbdev/uac1.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB Audio Class 1.0 playback gadget.  It exposes the host PCM stream as
 * /dev/uac1 so a small application can bridge it to any NuttX audio output.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/nuttx.h>
#include <nuttx/spinlock.h>

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include <nuttx/debug.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/usb/audio.h>
#include <nuttx/usb/uac1.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UAC1_DEVPATH              "/dev/uac1"
#define UAC1_CONFIGID             1
#define UAC1_AC_INTERFACE         0
#define UAC1_AS_INTERFACE         1
#define UAC1_FEATURE_UNIT         2
#define UAC1_PACKET_SIZE          192 /* 48 kHz * 2 ch * 16 bit / 1000 */
#define UAC1_CONFIG_DESC_SIZE     110
#define UAC1_CTRLREQ_SIZE         UAC1_CONFIG_DESC_SIZE

#define UAC1_REQ_SET_CUR          0x01
#define UAC1_REQ_GET_CUR          0x81
#define UAC1_REQ_GET_MIN          0x82
#define UAC1_REQ_GET_MAX          0x83
#define UAC1_REQ_GET_RES          0x84
#define UAC1_CONTROL_MUTE         0x01
#define UAC1_CONTROL_VOLUME       0x02

#define UAC1_VOLUME_MIN           (-63 * 256)
#define UAC1_VOLUME_MAX           0
#define UAC1_VOLUME_RES           256
#define UAC1_VOLUME_DEFAULT       (-10 * 256)

#define UAC1_REQ_IDLE             0
#define UAC1_REQ_SUBMITTED        1
#define UAC1_REQ_QUEUED           2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uac1_dev_s;

struct uac1_req_s
{
  sq_entry_t node;
  FAR struct usbdev_req_s *req;
  FAR struct uac1_dev_s *dev;
  size_t offset;
  uint8_t state;
};

struct uac1_dev_s
{
  FAR struct usbdev_s *usbdev;
  FAR struct usbdev_ep_s *epout;
  FAR struct usbdev_req_s *ctrlreq;
  struct uac1_req_s reqs[CONFIG_UAC1_NRDREQS];
  struct sq_queue_s rxqueue;
  mutex_t lock;
  sem_t rxsem;
  spinlock_t spinlock;
  uint8_t config;
  uint8_t alt;
  bool streaming;
  bool unlinked;
  bool mute;
  int16_t volume;
};

struct uac1_alloc_s
{
  struct uac1_dev_s dev;
  struct usbdevclass_driver_s drvr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int uac1_bind(FAR struct usbdevclass_driver_s *driver,
                     FAR struct usbdev_s *dev);
static void uac1_unbind(FAR struct usbdevclass_driver_s *driver,
                        FAR struct usbdev_s *dev);
static int uac1_setup(FAR struct usbdevclass_driver_s *driver,
                      FAR struct usbdev_s *dev,
                      FAR const struct usb_ctrlreq_s *ctrl,
                      FAR uint8_t *dataout, size_t outlen);
static void uac1_disconnect(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct usbdevclass_driverops_s g_uac1_driverops =
{
  .bind       = uac1_bind,
  .unbind     = uac1_unbind,
  .setup      = uac1_setup,
  .disconnect = uac1_disconnect,
};

static const struct usb_devdesc_s g_uac1_devdesc =
{
  .len          = USB_SIZEOF_DEVDESC,
  .type         = USB_DESC_TYPE_DEVICE,
  .usb          =
  {
    LSBYTE(0x0200), MSBYTE(0x0200)
  },
  .classid      = 0,
  .subclass     = 0,
  .protocol     = 0,
  .mxpacketsize = CONFIG_UAC1_EP0MAXPACKET,
  .vendor       =
  {
    LSBYTE(CONFIG_UAC1_VENDORID), MSBYTE(CONFIG_UAC1_VENDORID)
  },
  .product      =
  {
    LSBYTE(CONFIG_UAC1_PRODUCTID), MSBYTE(CONFIG_UAC1_PRODUCTID)
  },
  .device       =
  {
    LSBYTE(0x0100), MSBYTE(0x0100)
  },
  .imfgr        = 1,
  .iproduct     = 2,
  .serno        = 3,
  .nconfigs     = 1,
};

/* UAC1 speaker: USB streaming input -> feature unit -> speaker output.
 * The endpoint address at byte 96 is patched at runtime.
 */

static const uint8_t g_uac1_cfgdesc[UAC1_CONFIG_DESC_SIZE] =
{
  9, USB_DESC_TYPE_CONFIG, LSBYTE(UAC1_CONFIG_DESC_SIZE),
  MSBYTE(UAC1_CONFIG_DESC_SIZE), 2, 1, 4, USB_CONFIG_ATTR_ONE, 50,

  9, USB_DESC_TYPE_INTERFACE, 0, 0, 0, USB_CLASS_AUDIO,
  ADC_SUBCLASS_AUDIOCONTROL, 0, 5,
  9, ADC_CS_INTERFACE, 1, 0x00, 0x01, 0x28, 0x00, 1, 1,
  12, ADC_CS_INTERFACE, 2, 1, 0x01, 0x01, 0, 2, 0x03, 0x00, 0, 0,
  10, ADC_CS_INTERFACE, 6, UAC1_FEATURE_UNIT, 1, 1, 0x03, 0x03, 0x03, 0,
  9, ADC_CS_INTERFACE, 3, 3, 0x01, 0x03, 0, UAC1_FEATURE_UNIT, 0,

  9, USB_DESC_TYPE_INTERFACE, 1, 0, 0, USB_CLASS_AUDIO,
  ADC_SUBCLASS_AUDIOSTREAMING, 0, 6,
  9, USB_DESC_TYPE_INTERFACE, 1, 1, 1, USB_CLASS_AUDIO,
  ADC_SUBCLASS_AUDIOSTREAMING, 0, 6,
  7, ADC_CS_INTERFACE, 1, 1, 1, 0x01, 0x00,
  11, ADC_CS_INTERFACE, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00,
  9, USB_DESC_TYPE_ENDPOINT, 1, USB_EP_ATTR_XFER_ISOC |
  USB_EP_ATTR_ADAPTIVE | USB_EP_ATTR_USAGE_DATA,
  LSBYTE(UAC1_PACKET_SIZE), MSBYTE(UAC1_PACKET_SIZE), 1, 0, 0,
  7, ADC_CS_ENDPOINT, 1, 0, 0, 0, 0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct uac1_dev_s *uac1_from_driver(
    FAR struct usbdevclass_driver_s *driver)
{
  FAR struct uac1_alloc_s *alloc =
    container_of(driver, struct uac1_alloc_s, drvr);
  return &alloc->dev;
}

static void uac1_ep0complete(FAR struct usbdev_ep_s *ep,
                             FAR struct usbdev_req_s *req)
{
  if (req->result < 0)
    {
      uerr("UAC1 EP0 completion failed: %d\n", req->result);
    }
}

static int uac1_submit(FAR struct uac1_req_s *container)
{
  int ret;

  container->offset = 0;
  container->req->len = UAC1_PACKET_SIZE;
  container->req->flags = 0;
  container->state = UAC1_REQ_SUBMITTED;
  ret = EP_SUBMIT(container->dev->epout, container->req);
  if (ret < 0)
    {
      container->state = UAC1_REQ_IDLE;
    }

  return ret;
}

static void uac1_rxcomplete(FAR struct usbdev_ep_s *ep,
                            FAR struct usbdev_req_s *req)
{
  FAR struct uac1_req_s *container = req->priv;
  FAR struct uac1_dev_s *priv = container->dev;
  irqstate_t flags;

  container->state = UAC1_REQ_IDLE;
  if (req->result == OK && req->xfrd > 0 && priv->streaming)
    {
      container->offset = 0;
      flags = spin_lock_irqsave_nopreempt(&priv->spinlock);
      container->state = UAC1_REQ_QUEUED;
      sq_addlast(&container->node, &priv->rxqueue);
      spin_unlock_irqrestore_nopreempt(&priv->spinlock, flags);
    }
  else if (req->result == OK && priv->streaming)
    {
      uac1_submit(container);
      return;
    }

  nxsem_post(&priv->rxsem);
}

static void uac1_flush(FAR struct uac1_dev_s *priv)
{
  FAR struct uac1_req_s *container;
  irqstate_t flags;

  for (; ; )
    {
      flags = spin_lock_irqsave_nopreempt(&priv->spinlock);
      container = (FAR struct uac1_req_s *)sq_remfirst(&priv->rxqueue);
      if (container != NULL)
        {
          container->state = UAC1_REQ_IDLE;
        }

      spin_unlock_irqrestore_nopreempt(&priv->spinlock, flags);
      if (container == NULL)
        {
          break;
        }
    }
}

static void uac1_stop(FAR struct uac1_dev_s *priv)
{
  if (!priv->streaming)
    {
      return;
    }

  priv->streaming = false;
  priv->alt = 0;
  if (priv->epout != NULL)
    {
      EP_CANCEL(priv->epout, NULL);
      EP_DISABLE(priv->epout);
    }

  uac1_flush(priv);
  nxsem_post(&priv->rxsem);
}

static int uac1_start(FAR struct uac1_dev_s *priv)
{
  struct usb_epdesc_s epdesc;
  int ret;
  int i;

  if (priv->config != UAC1_CONFIGID)
    {
      return -EINVAL;
    }

  uac1_stop(priv);
  memset(&epdesc, 0, sizeof(epdesc));
  epdesc.len = USB_SIZEOF_EPDESC;
  epdesc.type = USB_DESC_TYPE_ENDPOINT;
  epdesc.addr = USB_EPOUT(USB_EPNO(priv->epout->eplog));
  epdesc.attr = USB_EP_ATTR_XFER_ISOC | USB_EP_ATTR_ADAPTIVE |
                USB_EP_ATTR_USAGE_DATA;
  epdesc.mxpacketsize[0] = LSBYTE(UAC1_PACKET_SIZE);
  epdesc.mxpacketsize[1] = MSBYTE(UAC1_PACKET_SIZE);
  epdesc.interval = 1;

  ret = EP_CONFIGURE(priv->epout, &epdesc, true);
  if (ret < 0)
    {
      return ret;
    }

  priv->alt = 1;
  priv->streaming = true;
  for (i = 0; i < CONFIG_UAC1_NRDREQS; i++)
    {
      if (priv->reqs[i].state == UAC1_REQ_IDLE)
        {
          ret = uac1_submit(&priv->reqs[i]);
          if (ret < 0)
            {
              uac1_stop(priv);
              return ret;
            }
        }
    }

  nxsem_post(&priv->rxsem);
  return OK;
}

static int uac1_mkstrdesc(uint8_t id, FAR struct usb_strdesc_s *desc)
{
  FAR uint8_t *data = (FAR uint8_t *)(desc + 1);
  FAR const char *str;
  size_t len;
  int i;

  switch (id)
    {
      case 0:
        desc->len = 4;
        desc->type = USB_DESC_TYPE_STRING;
        data[0] = 0x09;
        data[1] = 0x04;
        return 4;
      case 1:
        str = CONFIG_UAC1_VENDORSTR;
        break;
      case 2:
        str = CONFIG_UAC1_PRODUCTSTR;
        break;
      case 3:
        str = CONFIG_UAC1_SERIALSTR;
        break;
      case 4:
        str = "Audio configuration";
        break;
      case 5:
        str = "Audio control";
        break;
      case 6:
        str = "Audio streaming";
        break;
      default:
        return -EINVAL;
    }

  len = strlen(str);
  if (len > 31)
    {
      len = 31;
    }

  for (i = 0; i < len; i++)
    {
      data[i * 2] = str[i];
      data[i * 2 + 1] = 0;
    }

  desc->len = len * 2 + 2;
  desc->type = USB_DESC_TYPE_STRING;
  return desc->len;
}

static int uac1_control(FAR struct uac1_dev_s *priv,
                        FAR const struct usb_ctrlreq_s *ctrl,
                        FAR uint8_t *dataout, size_t outlen)
{
  uint8_t selector = ctrl->value[1];
  uint8_t unit = ctrl->index[1];
  uint16_t len = GETUINT16(ctrl->len);
  int16_t value;

  if (unit != UAC1_FEATURE_UNIT)
    {
      return -EOPNOTSUPP;
    }

  if (ctrl->req == UAC1_REQ_SET_CUR)
    {
      if (dataout == NULL)
        {
          return 0; /* The controller will call setup again with dataout. */
        }

      if (selector == UAC1_CONTROL_MUTE && outlen >= 1)
        {
          priv->mute = dataout[0] != 0;
          return 0;
        }

      if (selector == UAC1_CONTROL_VOLUME && outlen >= 2)
        {
          value = (int16_t)((uint16_t)dataout[0] |
                            ((uint16_t)dataout[1] << 8));
          if (value < UAC1_VOLUME_MIN)
            {
              value = UAC1_VOLUME_MIN;
            }
          else if (value > UAC1_VOLUME_MAX)
            {
              value = UAC1_VOLUME_MAX;
            }

          priv->volume = value;
          return 0;
        }

      return -EINVAL;
    }

  if (selector == UAC1_CONTROL_MUTE && ctrl->req == UAC1_REQ_GET_CUR)
    {
      priv->ctrlreq->buf[0] = priv->mute;
      return len < 1 ? len : 1;
    }

  if (selector == UAC1_CONTROL_VOLUME)
    {
      switch (ctrl->req)
        {
          case UAC1_REQ_GET_CUR:
            value = priv->volume;
            break;
          case UAC1_REQ_GET_MIN:
            value = UAC1_VOLUME_MIN;
            break;
          case UAC1_REQ_GET_MAX:
            value = UAC1_VOLUME_MAX;
            break;
          case UAC1_REQ_GET_RES:
            value = UAC1_VOLUME_RES;
            break;
          default:
            return -EOPNOTSUPP;
        }

      priv->ctrlreq->buf[0] = value & 0xff;
      priv->ctrlreq->buf[1] = ((uint16_t)value >> 8) & 0xff;
      return len < 2 ? len : 2;
    }

  return -EOPNOTSUPP;
}

static int uac1_bind(FAR struct usbdevclass_driver_s *driver,
                     FAR struct usbdev_s *dev)
{
  FAR struct uac1_dev_s *priv = uac1_from_driver(driver);
  int i;

  priv->usbdev = dev;
  priv->ctrlreq = usbdev_allocreq(dev->ep0, UAC1_CTRLREQ_SIZE);
  if (priv->ctrlreq == NULL)
    {
      return -ENOMEM;
    }

  priv->ctrlreq->callback = uac1_ep0complete;
  priv->epout = DEV_ALLOCEP(dev, CONFIG_UAC1_EPOUT, false,
                            USB_EP_ATTR_XFER_ISOC);
  if (priv->epout == NULL)
    {
      usbdev_freereq(dev->ep0, priv->ctrlreq);
      priv->ctrlreq = NULL;
      return -ENODEV;
    }

  priv->epout->priv = priv;
  for (i = 0; i < CONFIG_UAC1_NRDREQS; i++)
    {
      priv->reqs[i].req = usbdev_allocreq(priv->epout, UAC1_PACKET_SIZE);
      if (priv->reqs[i].req == NULL)
        {
          uac1_unbind(driver, dev);
          return -ENOMEM;
        }

      priv->reqs[i].dev = priv;
      priv->reqs[i].req->priv = &priv->reqs[i];
      priv->reqs[i].req->callback = uac1_rxcomplete;
    }

  DEV_CONNECT(dev);
  return OK;
}

static void uac1_unbind(FAR struct usbdevclass_driver_s *driver,
                        FAR struct usbdev_s *dev)
{
  FAR struct uac1_dev_s *priv = uac1_from_driver(driver);
  int i;

  uac1_stop(priv);
  priv->unlinked = true;
  nxsem_post(&priv->rxsem);

  if (priv->epout != NULL)
    {
      for (i = 0; i < CONFIG_UAC1_NRDREQS; i++)
        {
          if (priv->reqs[i].req != NULL)
            {
              usbdev_freereq(priv->epout, priv->reqs[i].req);
              priv->reqs[i].req = NULL;
            }
        }

      DEV_FREEEP(dev, priv->epout);
      priv->epout = NULL;
    }

  if (priv->ctrlreq != NULL)
    {
      usbdev_freereq(dev->ep0, priv->ctrlreq);
      priv->ctrlreq = NULL;
    }

  DEV_DISCONNECT(dev);
}

static int uac1_setup(FAR struct usbdevclass_driver_s *driver,
                      FAR struct usbdev_s *dev,
                      FAR const struct usb_ctrlreq_s *ctrl,
                      FAR uint8_t *dataout, size_t outlen)
{
  FAR struct uac1_dev_s *priv = uac1_from_driver(driver);
  FAR struct usbdev_req_s *req = priv->ctrlreq;
  uint16_t value = GETUINT16(ctrl->value);
  uint16_t index = GETUINT16(ctrl->index);
  uint16_t len = GETUINT16(ctrl->len);
  int ret = -EOPNOTSUPP;

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      ret = uac1_control(priv, ctrl, dataout, outlen);
      if (ret == 0 && ctrl->req == UAC1_REQ_SET_CUR && dataout == NULL &&
          len > 0)
        {
          return 0;
        }
    }
  else if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      switch (ctrl->req)
        {
          case USB_REQ_GETDESCRIPTOR:
            switch (ctrl->value[1])
              {
                case USB_DESC_TYPE_DEVICE:
                  ret = usbdev_copy_devdesc(req->buf, &g_uac1_devdesc,
                                            dev->speed);
                  break;
                case USB_DESC_TYPE_CONFIG:
                  memcpy(req->buf, g_uac1_cfgdesc, UAC1_CONFIG_DESC_SIZE);
                  req->buf[96] = USB_EPOUT(USB_EPNO(priv->epout->eplog));
                  ret = UAC1_CONFIG_DESC_SIZE;
                  break;
                case USB_DESC_TYPE_STRING:
                  ret = uac1_mkstrdesc(ctrl->value[0],
                                      (FAR struct usb_strdesc_s *)req->buf);
                  break;
                default:
                  break;
              }
            break;

          case USB_REQ_SETCONFIGURATION:
            if (value == 0)
              {
                uac1_stop(priv);
                priv->config = 0;
                ret = 0;
              }
            else if (value == UAC1_CONFIGID)
              {
                priv->config = UAC1_CONFIGID;
                ret = 0;
              }
            break;

          case USB_REQ_GETCONFIGURATION:
            req->buf[0] = priv->config;
            ret = 1;
            break;

          case USB_REQ_SETINTERFACE:
            if ((index & 0xff) == UAC1_AS_INTERFACE && value <= 1)
              {
                ret = value == 1 ? uac1_start(priv) : (uac1_stop(priv), OK);
              }
            else if ((index & 0xff) == UAC1_AC_INTERFACE && value == 0)
              {
                ret = 0;
              }
            break;

          case USB_REQ_GETINTERFACE:
            if ((index & 0xff) == UAC1_AS_INTERFACE)
              {
                req->buf[0] = priv->alt;
                ret = 1;
              }
            else if ((index & 0xff) == UAC1_AC_INTERFACE)
              {
                req->buf[0] = 0;
                ret = 1;
              }
            break;

          default:
            break;
        }
    }

  if (ret >= 0)
    {
      req->len = len < ret ? len : ret;
      req->flags = USBDEV_REQFLAGS_NULLPKT;
      ret = EP_SUBMIT(dev->ep0, req);
    }

  return ret;
}

static void uac1_disconnect(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
  FAR struct uac1_dev_s *priv = uac1_from_driver(driver);

  priv->config = 0;
  uac1_stop(priv);
}

static int uac1_open(FAR struct file *filep)
{
  return OK;
}

static int uac1_close(FAR struct file *filep)
{
  return OK;
}

static ssize_t uac1_read(FAR struct file *filep, FAR char *buffer,
                         size_t len)
{
  FAR struct uac1_dev_s *priv = filep->f_inode->i_private;
  FAR struct uac1_req_s *container;
  irqstate_t flags;
  size_t copied = 0;
  size_t available;
  size_t chunk;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  while (copied < len)
    {
      flags = spin_lock_irqsave_nopreempt(&priv->spinlock);
      container = (FAR struct uac1_req_s *)sq_peek(&priv->rxqueue);
      spin_unlock_irqrestore_nopreempt(&priv->spinlock, flags);

      if (container == NULL)
        {
          if (!priv->streaming || priv->unlinked)
            {
              ret = copied > 0 ? (int)copied : -EPIPE;
              goto out;
            }

          if ((filep->f_oflags & O_NONBLOCK) != 0)
            {
              ret = copied > 0 ? (int)copied : -EAGAIN;
              goto out;
            }

          nxmutex_unlock(&priv->lock);
          ret = nxsem_wait(&priv->rxsem);
          if (ret < 0)
            {
              return ret;
            }

          ret = nxmutex_lock(&priv->lock);
          if (ret < 0)
            {
              return ret;
            }

          continue;
        }

      available = container->req->xfrd - container->offset;
      chunk = len - copied;
      if (chunk > available)
        {
          chunk = available;
        }

      memcpy(buffer + copied, container->req->buf + container->offset,
             chunk);
      copied += chunk;
      container->offset += chunk;

      if (container->offset == container->req->xfrd)
        {
          flags = spin_lock_irqsave_nopreempt(&priv->spinlock);
          sq_remfirst(&priv->rxqueue);
          container->state = UAC1_REQ_IDLE;
          spin_unlock_irqrestore_nopreempt(&priv->spinlock, flags);
          if (priv->streaming)
            {
              uac1_submit(container);
            }
        }
    }

  ret = copied;

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

static int uac1_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct uac1_dev_s *priv = filep->f_inode->i_private;

  if (cmd == UAC1IOC_GETSTATUS)
    {
      FAR struct uac1_status_s *status =
        (FAR struct uac1_status_s *)(uintptr_t)arg;
      if (status == NULL)
        {
          return -EINVAL;
        }

      status->volume = priv->volume;
      status->mute = priv->mute;
      status->configured = priv->config == UAC1_CONFIGID;
      status->streaming = priv->streaming;
      return OK;
    }

  return -ENOTTY;
}

static const struct file_operations g_uac1_fops =
{
  .open  = uac1_open,
  .close = uac1_close,
  .read  = uac1_read,
  .ioctl = uac1_ioctl,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *usbdev_uac1_initialize(void)
{
  FAR struct uac1_alloc_s *alloc;
  FAR struct uac1_dev_s *priv;
  int ret;

  alloc = kmm_zalloc(sizeof(*alloc));
  if (alloc == NULL)
    {
      return NULL;
    }

  priv = &alloc->dev;
  sq_init(&priv->rxqueue);
  nxmutex_init(&priv->lock);
  nxsem_init(&priv->rxsem, 0, 0);
  spin_lock_init(&priv->spinlock);
  priv->volume = UAC1_VOLUME_DEFAULT;
  alloc->drvr.ops = &g_uac1_driverops;
  alloc->drvr.speed = USB_SPEED_FULL;

  ret = register_driver(UAC1_DEVPATH, &g_uac1_fops, 0444, priv);
  if (ret < 0)
    {
      goto errout;
    }

  ret = usbdev_register(&alloc->drvr);
  if (ret < 0)
    {
      unregister_driver(UAC1_DEVPATH);
      goto errout;
    }

  return alloc;

errout:
  nxsem_destroy(&priv->rxsem);
  nxmutex_destroy(&priv->lock);
  kmm_free(alloc);
  return NULL;
}

void usbdev_uac1_uninitialize(FAR void *handle)
{
  FAR struct uac1_alloc_s *alloc = handle;

  if (alloc != NULL)
    {
      usbdev_unregister(&alloc->drvr);
      unregister_driver(UAC1_DEVPATH);
      nxsem_destroy(&alloc->dev.rxsem);
      nxmutex_destroy(&alloc->dev.lock);
      kmm_free(alloc);
    }
}
