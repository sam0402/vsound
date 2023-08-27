#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include "vsound.h"

MODULE_AUTHOR("K.Yoshioka");
MODULE_DESCRIPTION("Audio over Ether Virtual soundcard");
MODULE_LICENSE("GPL");

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,0,0))
#define _STATE_	state
#else
#define _STATE_	status->state
#endif

#define DRIVER_NAME			"snd_vsound"
#define MAX_PCM_DEVICES		1
#define MAX_PCM_SUBSTREAMS	128
#define MIXER_ADDR_DIGITAL	0

static struct platform_device *devices[SNDRV_CARDS];
static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;	/* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;	/* ID for this card */
static bool enable[SNDRV_CARDS] = {1, [1 ... (SNDRV_CARDS - 1)] = 0};
static int pcm_devs[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 1};
static int pcm_substreams[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = 1};

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for virtual soundcard.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for virtual soundcard.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable this virtual soundcard.");
module_param_array(pcm_devs, int, NULL, 0444);
MODULE_PARM_DESC(pcm_devs, "PCM devices # (0-4) for vsound driver.");
module_param_array(pcm_substreams, int, NULL, 0444);
MODULE_PARM_DESC(pcm_substreams, "PCM substreams # (1-128) for vsound driver.");

static int period_bytes_min = 371;
static int periods_min = 4;
module_param(period_bytes_min, int, S_IRUSR | S_IWUSR);
module_param(periods_min, int, S_IRUSR | S_IWUSR);

/* logging */
#ifdef DEBUG_VSOUND
#define LOG(fmt, ...) printk(KERN_INFO fmt, ##__VA_ARGS__)
#define STATE(v) [SNDRV_PCM_STATE_##v] = #v
static char *snd_pcm_state_names[] = {
	STATE(OPEN),
	STATE(SETUP),
	STATE(PREPARED),
	STATE(RUNNING),
	STATE(XRUN),
	STATE(DRAINING),
	STATE(PAUSED),
	STATE(SUSPENDED),
	STATE(DISCONNECTED),
};
static const char *snd_pcm_state_name(snd_pcm_state_t state)
{
	return snd_pcm_state_names[(__force int)state];
}
#else
#define LOG(fmt, ...)
#endif

struct snd_vsound {
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_pcm_hardware pcm_hw;
	spinlock_t mixer_lock;
};
struct active_buffer {
	struct snd_pcm_substream *substream;
	size_t size;		/* frame */
	unsigned int tail;	/* bytes */
	bool state_change_notify;
};
static struct active_buffer act;
static struct vsound_pcm pcm;
static struct vsound_model backend_model = (struct vsound_model){ 0 };

/* default model */
static const struct snd_pcm_hardware vsound_pcm_hardware = {
//	.info =			(SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
//				 SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.info =			(SNDRV_PCM_INFO_INTERLEAVED),
	.formats =		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates =		(SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000|
				SNDRV_PCM_RATE_88200|SNDRV_PCM_RATE_96000|
				SNDRV_PCM_RATE_176400|SNDRV_PCM_RATE_192000|
#ifdef CONFIG_SMPD_OPTION_RPI_DAC_32BIT_786KHZ
				SNDRV_PCM_RATE_352800|SNDRV_PCM_RATE_384000|SNDRV_PCM_RATE_705600|SNDRV_PCM_RATE_768000),
#else
				SNDRV_PCM_RATE_352800|SNDRV_PCM_RATE_384000),
#endif
	.rate_min =		44100,
#ifdef CONFIG_SMPD_OPTION_RPI_DAC_32BIT_786KHZ
	.rate_max =		768000,
#else
	.rate_max =		384000,
#endif
	.channels_min =		2,
	.channels_max =		2,
	.buffer_bytes_max =	1024*2048,
	.period_bytes_max =	65536,
	.period_bytes_min =	371,
	.periods_min =		4,
	.periods_max =		256,
	.fifo_size =		0,
};

/*
 * PCM interface
 */
static int vsound_pcm_open(struct snd_pcm_substream *substream)
{
	memcpy(pcm.comm, current->comm, sizeof(pcm.comm));
	LOG("OPEN (id:%d vid:%d %s)", (int)task_pid_nr(current), (int)task_pid_vnr(current), pcm.comm);

	struct snd_vsound *vsound = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	/*
	 * If the backend_model has been updated, apply the changes to pcm_hw.
	 * The backend_model is updated through ioctl.
	 */
	if (backend_model.rates > 0) {
		vsound->pcm_hw.formats = backend_model.formats;
		vsound->pcm_hw.rates = backend_model.rates;
		vsound->pcm_hw.rate_min = backend_model.rate_min;
		vsound->pcm_hw.rate_max = backend_model.rate_max;
		vsound->pcm_hw.channels_min = backend_model.channels_min;
		vsound->pcm_hw.channels_max = backend_model.channels_max;
		strcpy(vsound->card->longname, backend_model.name);
	}

	/* Apply the parameters set by ioctl. */
	vsound->pcm_hw.period_bytes_min = period_bytes_min;
	vsound->pcm_hw.periods_min = periods_min;

	runtime->hw = vsound->pcm_hw;
	return 0;
}

static int vsound_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	/* non-atomic (schedulable) */
	/* this callback may be called multiple times */
	pcm.state	= substream->runtime->_STATE_;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,0,0))
	return 0;
#else
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
#endif
}

static int vsound_pcm_prepare(struct snd_pcm_substream *substream)
{
	/* non-atomic (schedulable) */
	/* this callback may be called multiple times. */
	/* In this callback, it can refer to runtime record. */

	struct snd_pcm_runtime *runtime = substream->runtime;
	LOG("PREPARE (%d)", runtime->_STATE_);
	act.substream	= substream;
	act.tail	= 0;
	act.size	= 0;
	act.state_change_notify = false;
	pcm.state	= runtime->_STATE_;

	/* If there are any changes in sample rate or other parameters, set the notification flag. */
	if (pcm.format != runtime->format
		|| pcm.rate != runtime->rate
		|| pcm.channels != runtime->channels) {
		act.state_change_notify = true;
	}
	pcm.format	= runtime->format;
	pcm.rate	= runtime->rate;
	pcm.channels	= runtime->channels;

	pcm.buffer_bytes = frames_to_bytes(runtime, runtime->buffer_size);
	LOG("%s %u %u", snd_pcm_format_name(runtime->format), runtime->rate, runtime->channels);
	LOG("period_size:%lu (%ldB) periods:%u buffer_size:%lu (%ldB) frame_bits:%u dma_bytes:%lu",
		runtime->period_size, frames_to_bytes(runtime, runtime->period_size),
		runtime->periods,
		runtime->buffer_size, pcm.buffer_bytes,
		runtime->frame_bits, runtime->dma_bytes);
	return 0;
}

static int vsound_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	/* atomic (it cannot call functions which may sleep) */
	/* In this callback, it can refer to runtime record. */
	/* trigger callback should be as minimal as possible. */

//	pcm.state = substream->runtime->_STATE_;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		LOG("pcm_trigger (%s)", snd_pcm_state_name(pcm.state));
		return 0;
	}
	return -EINVAL;
}

static snd_pcm_uframes_t vsound_pcm_pointer(struct snd_pcm_substream *substream)
{
	/* Return current hardware position on the buffer. */
	/* The position must be returned in frames. (0 to buffer_size - 1) */
	/* This callback is atomic, it cannot call functions which may sleep. */
	/* This is invoked when snd_pcm_period_elapsed() is called. */
	return bytes_to_frames(act.substream->runtime, act.tail);
}

static int vsound_pcm_hw_free(struct snd_pcm_substream *substream)
{
	LOG("HW FREE");
	act.substream	= NULL;
	act.tail	= 0;
	act.size	= 0;
	act.state_change_notify = false;
	pcm.state	= substream->runtime->_STATE_;

	/* Some playback software may perform CLOSE and then reOPEN between songs.
	 * In this case, if there are no changes in format or sample rate,
	 * there is no need for notification to aoecli. Therefore, initialization of format,
	 * sample rate, and channels will not be performed. */
//	pcm.format	= 0;
//	pcm.rate	= 0;
//	pcm.channels	= 0;
//	pcm.buffer_bytes = 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,0,0))
	return 0;
#else
	return snd_pcm_lib_free_pages(substream);
#endif
}

static int vsound_pcm_close(struct snd_pcm_substream *substream)
{
	LOG("CLOSE");
	/* private data */
	struct vsound_data *data;
	data = substream->runtime->private_data;
	if (data)
		kfree(data);

	pcm.state		= substream->runtime->_STATE_;
	act.substream	= NULL;
	return 0;
}

static struct snd_pcm_ops vsound_pcm_ops = {
	.open	=	vsound_pcm_open,
	.close	=	vsound_pcm_close,
	.ioctl	=	snd_pcm_lib_ioctl,
	.hw_params =	vsound_pcm_hw_params,
	.hw_free =	vsound_pcm_hw_free,
	.prepare =	vsound_pcm_prepare,
	.trigger =	vsound_pcm_trigger,
	.pointer =	vsound_pcm_pointer,
};

static int snd_card_vsound_pcm(struct snd_vsound *vsound, int device,
			      int substreams)
{
	struct snd_pcm *sndpcm;
	struct snd_pcm_ops *ops;
	int err;

	/* index = 0 and playback only */
	err = snd_pcm_new(vsound->card, "AoE VSOUND", 0, 1, 0, &sndpcm);
	if (err < 0)
		return err;
	vsound->pcm = sndpcm;
	ops = &vsound_pcm_ops;
	snd_pcm_set_ops(sndpcm, SNDRV_PCM_STREAM_PLAYBACK, ops);

	/* pre-allocation of buffers */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,0,0))
	snd_pcm_set_managed_buffer_all(sndpcm, SNDRV_DMA_TYPE_VMALLOC, NULL, 0, 0);
#else
	snd_pcm_lib_preallocate_pages_for_all(sndpcm,
		SNDRV_DMA_TYPE_CONTINUOUS,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5,4,0))
		NULL,
#else
		snd_dma_continuous_data(GFP_KERNEL),
#endif
		0, 1024*2048);
#endif
	sndpcm->private_data = vsound;
	sndpcm->info_flags = 0;
	strcpy(sndpcm->name, "Audio over Ether Virtual sound card");

	return 0;
}

/*
 * mixer interface
 */

/* hardware volume (Digital) */
static int snd_vsound_volume_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 207;
	return 0;
}
static int snd_vsound_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_vsound *vsound = snd_kcontrol_chip(kcontrol);
	struct vsound_control p = get_vsound_control(&pcm.mixer_value);
	spin_lock_irq(&vsound->mixer_lock);
	ucontrol->value.integer.value[0] = p.volume;
	spin_unlock_irq(&vsound->mixer_lock);
	return 0;
}
static int snd_vsound_volume_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int change;
	int vol;
	struct snd_vsound *vsound = snd_kcontrol_chip(kcontrol);
	vol = ucontrol->value.integer.value[0];
	if (vol < 0)
		vol = 0;
	if (vol > 255)
		vol = 255;
	struct vsound_control p = get_vsound_control(&pcm.mixer_value);
	spin_lock_irq(&vsound->mixer_lock);
	change = p.volume != vol;
	p.volume = vol;
	spin_unlock_irq(&vsound->mixer_lock);
	set_vsound_control(&pcm.mixer_value, p);
	return change;
}
static const DECLARE_TLV_DB_SCALE(db_scale_vsound, -10350, 50, 0);
static struct snd_kcontrol_new snd_vsound_controls = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE | SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.name = "Digital Playback Volume",
	.index = 0,
	.info = snd_vsound_volume_info,
	.get = snd_vsound_volume_get,
	.put = snd_vsound_volume_put,
	.private_value = MIXER_ADDR_DIGITAL,
	.tlv = { .p = db_scale_vsound }
};

static int snd_card_vsound_new_mixer(struct snd_vsound *vsound)
{
	struct snd_card *card = vsound->card;
	int err;
	spin_lock_init(&vsound->mixer_lock);
	strcpy(card->mixername, "AoE VSOUND Mixer");

	if ((err = snd_ctl_add(card, snd_ctl_new1(&snd_vsound_controls, vsound))) < 0)
		return err;
	return 0;
}

static int vsound_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct snd_vsound *vsound;
	int idx, err;
	int dev = devptr->id;

	err = snd_card_new(&devptr->dev, index[dev], id[dev], THIS_MODULE,
			   sizeof(struct snd_vsound), &card);
	if (err < 0)
		return err;
	vsound = card->private_data;
	vsound->card = card;

	for (idx = 0; idx < MAX_PCM_DEVICES && idx < pcm_devs[dev]; idx++) {
		if (pcm_substreams[dev] < 1)
			pcm_substreams[dev] = 1;
		if (pcm_substreams[dev] > MAX_PCM_SUBSTREAMS)
			pcm_substreams[dev] = MAX_PCM_SUBSTREAMS;
		err = snd_card_vsound_pcm(vsound, idx, pcm_substreams[dev]);
		if (err < 0)
			goto __nodev;
	}

	vsound->pcm_hw = vsound_pcm_hardware;
	err = snd_card_vsound_new_mixer(vsound);
	if (err < 0)
		goto __nodev;
	strcpy(card->driver, "vsound");
	strcpy(card->shortname, "vsound");
	sprintf(card->longname, "(Detecting the backend sound card...)");

	err = snd_card_register(card);
	if (err == 0) {
		platform_set_drvdata(devptr, card);
		return 0;
	}
      __nodev:
	snd_card_free(card);
	return err;
}
static int vsound_remove(struct platform_device *devptr)
{
	snd_card_free(platform_get_drvdata(devptr));
	return 0;
}
static struct platform_driver snd_vsound_driver = {
	.probe		= vsound_probe,
	.remove		= vsound_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.pm	= NULL,
	},
};

static void snd_vsound_unregister_all(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(devices); ++i)
		platform_device_unregister(devices[i]);
	platform_driver_unregister(&snd_vsound_driver);
}

/* char device */
#include <asm/uaccess.h>   /* copy_from_user, copy_to_user */
#include <linux/cdev.h>
#define DEVICE_NAME	"vsound"
static dev_t start;
static int major, minor;
static struct cdev* chardev = NULL;
static struct class *dev_class = NULL;
static int vsound_buffer_open(struct inode* inode __attribute__ ((unused)), struct file* filp __attribute__ ((unused)))
{
	return 0;
}

static ssize_t vsound_buffer_read(struct file* filp, char* buf, size_t count, loff_t* pos __attribute__ ((unused)))
{
	/* read stats, return avail bytes */
	if (count == 0) {

		/* Once the status reading is executed, release the notify flag. */
		act.state_change_notify = false;

		ssize_t ret = 0;
		if (act.substream) {
			struct snd_pcm_runtime *runtime = act.substream->runtime;
			snd_pcm_sframes_t avail = snd_pcm_playback_hw_avail(runtime);
			ret = frames_to_bytes(runtime, avail);
			pcm.state = runtime->_STATE_;
		}
		copy_to_user(buf, &pcm, sizeof(struct vsound_pcm));
		return ret;
	}

	/* buffer read */
	if (act.state_change_notify)
		return -ECANCELED;

	if (act.substream == NULL)
		return -EAGAIN;

	struct snd_pcm_runtime *runtime = act.substream->runtime;
	/* state check */
	if (runtime->_STATE_ != SNDRV_PCM_STATE_RUNNING &&
		runtime->_STATE_ != SNDRV_PCM_STATE_DRAINING) {
		return -EAGAIN;
	}

	/* buffer check */
	unsigned int tail = act.tail;
	unsigned int buffer_bytes = frames_to_bytes(runtime, runtime->buffer_size);

	snd_pcm_sframes_t avail = snd_pcm_playback_hw_avail(runtime);
	if (likely(avail > 0)) {
		avail = frames_to_bytes(runtime, avail);
	} else {
		if (runtime->_STATE_ == SNDRV_PCM_STATE_DRAINING) {
			return 0;
		} else {
			return -EAGAIN;
		}
	}
	if (unlikely(count > avail)) {
		count = avail;
	}

	if (tail + count >= buffer_bytes) {
		copy_to_user(buf, runtime->dma_area + tail, buffer_bytes - tail);
		copy_to_user(buf + buffer_bytes - tail, runtime->dma_area, tail + count - buffer_bytes);
		act.tail = tail + count - buffer_bytes;
	} else {
		copy_to_user(buf, runtime->dma_area + tail, count);
		act.tail += count;
		if (act.tail >= buffer_bytes)
			act.tail -= buffer_bytes;
	}

	/* period elapsed */
	act.size += bytes_to_frames(runtime, count);
	if (act.size >= runtime->period_size) {
		act.size %= runtime->period_size;
		snd_pcm_period_elapsed(act.substream);
	}

	return count;
}
static int vsound_buffer_release(struct inode* inode __attribute__ ((unused)), struct file* filp __attribute__ ((unused)))
{
	return 0;
}
static long vsound_buffer_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long flags;

	switch (cmd) {
	case IOCTL_VSOUND_STOP_IMMEDIATELY:
		LOG("ioctl... PCM Stop");
		if (act.substream) {
			/* pcm stop */
			snd_pcm_stream_lock_irqsave(act.substream, flags);
			if (snd_pcm_running(act.substream)) {
				snd_pcm_stop(act.substream, SNDRV_PCM_STATE_DISCONNECTED);
			}
			snd_pcm_stream_unlock_irqrestore(act.substream, flags);
		}
		break;
	case IOCTL_VSOUND_APPLY_MODEL:
		if (copy_from_user(&backend_model, (void __user *)arg, sizeof(struct vsound_model)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
const struct file_operations fops = {
	owner		: THIS_MODULE,
	open		: vsound_buffer_open,
	release		: vsound_buffer_release,
	read		: vsound_buffer_read,
	unlocked_ioctl	: vsound_buffer_ioctl,
};

static int __init alsa_card_vsound_init(void)
{
	int i, cards, err;
	err = platform_driver_register(&snd_vsound_driver);
	if (err < 0)
		return err;

	cards = 0;
	for (i = 0; i < SNDRV_CARDS; i++) {
		struct platform_device *device;
		if (! enable[i])
			continue;
		device = platform_device_register_simple(DRIVER_NAME, i, NULL, 0);
		if (IS_ERR(device))
			continue;
		if (!platform_get_drvdata(device)) {
			platform_device_unregister(device);
			continue;
		}
		devices[i] = device;
		cards++;
	}
	if (!cards) {
		printk(KERN_ERR "AoE virtual soundcard not found or device busy\n");
		snd_vsound_unregister_all();
		return -ENODEV;
	}

	/* chardev */
	err = alloc_chrdev_region(&start, 0, 1, DEVICE_NAME);
	if (err) {
		printk(KERN_ERR "Couldn't create the /dev entry \n");
		snd_vsound_unregister_all();	/* vsound */
		return -ENOMEM;
	} else {
		if((chardev = cdev_alloc()) == NULL) return -ENOMEM;
		cdev_init(chardev,&fops);
		if((err = cdev_add(chardev,start,1))) return -ENOMEM;
		major = MAJOR(start);
		minor = MINOR(start);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,0))
		dev_class = class_create(DEVICE_NAME);
#else
		dev_class = class_create(THIS_MODULE, DEVICE_NAME);
#endif
		if (IS_ERR(dev_class)) {
			cdev_del(chardev);
			unregister_chrdev_region( start, 1 );	/* chardev */
			snd_vsound_unregister_all();		/* vsound */
			return -ENODEV;
		}
		device_create(dev_class, NULL, MKDEV(major, minor), NULL, DEVICE_NAME);
	}

	return 0;
}

static void __exit alsa_card_vsound_exit(void)
{
	/* vsound */
	snd_vsound_unregister_all();

	/* chardev */
	if (dev_class) {
		device_destroy(dev_class, MKDEV(major, minor));
		class_destroy(dev_class);
	}
	if (chardev) cdev_del(chardev);
	unregister_chrdev_region(start, 1);
}

module_init(alsa_card_vsound_init)
module_exit(alsa_card_vsound_exit)
