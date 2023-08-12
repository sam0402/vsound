/* IOCTL */
#define IOCTL_VSOUND_STOP_IMMEDIATELY	_IO('v', 0)
#define IOCTL_VSOUND_APPLY_MODEL 		_IOW('v', 1, struct vsound_model)

struct vsound_pcm {
	snd_pcm_state_t state;
	snd_pcm_format_t format;	/* SNDRV_PCM_FORMAT_* */
	unsigned int rate;			/* rate in Hz */
	unsigned int channels;		/* channels */
	unsigned int period_us;		/* period time (us), server only used */
	ssize_t buffer_bytes;
	uint32_t mixer_value;
};

struct vsound_model {
	unsigned int info;			/* SNDRV_PCM_INFO_* */
	unsigned long long formats;	/* SNDRV_PCM_FMTBIT_* */
	unsigned int rates;			/* SNDRV_PCM_RATE_* */
	unsigned int rate_min;		/* min rate */
	unsigned int rate_max;		/* max rate */
	unsigned int channels_min;	/* min channels */
	unsigned int channels_max;	/* max channels */
	size_t buffer_bytes_max;	/* max buffer size */
	size_t period_bytes_min;	/* min period size */
	size_t period_bytes_max;	/* max period size */
	unsigned int periods_min;	/* min # of periods */
	unsigned int periods_max;	/* max # of periods */
	size_t fifo_size;		/* fifo size in bytes */
	char name[64];
};

struct vsound_control {
	int volume;
//	int dreq;
//	int recv;
};
static inline struct vsound_control get_vsound_control(uint32_t *mixer_value)
{
	struct vsound_control ret;
	ret.volume = *mixer_value & 0xff;
//	ret.dreq = (*mixer_value >> 8) & 0xff;
//	ret.recv = (*mixer_value >> 16) & 0xff;
	return ret;
}
static inline void set_vsound_control(uint32_t *mixer_value, struct vsound_control p)
{
//	*mixer_value = p.volume | (p.dreq << 8) | (p.recv << 16);
	*mixer_value = p.volume;
}

