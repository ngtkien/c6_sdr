/* ES8311 speaker output — init + fire-and-forget PCM writes onto i2s0.
 * Pattern follows projects/esp32p4-ai-hub/src/beep.cpp: codec as clock
 * slave, i2s0 as master, silence preloaded so TX stays armed. */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <zephyr/logging/log.h>

#include <bsp/esp32p4_bsp.h>

#include "audio.h"

#if CONFIG_I2S

LOG_MODULE_REGISTER(audio, LOG_LEVEL_INF);

#define SAMPLE_RATE   48000
#define WORD_SIZE     16
#define NUM_CHANNELS  2
#define BLOCK_FRAMES  1024
#define NUM_BLOCKS    4
#define BLOCK_BYTES   (BLOCK_FRAMES * NUM_CHANNELS * sizeof(int16_t))

K_MEM_SLAB_DEFINE_STATIC(audio_slab, BLOCK_BYTES, NUM_BLOCKS, 4);

static const struct device *i2s_dev;
static bool ok;

int audio_init(void)
{
	const struct device *codec = DEVICE_DT_GET(DT_NODELABEL(es8311));

	i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
	if (!device_is_ready(codec) || !device_is_ready(i2s_dev)) {
		LOG_WRN("audio devices not ready");
		return -ENODEV;
	}

	audio_dai_cfg_t dai = {};
	dai.i2s.word_size = WORD_SIZE;
	dai.i2s.channels = NUM_CHANNELS;
	dai.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
	dai.i2s.options = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE;
	dai.i2s.frame_clk_freq = SAMPLE_RATE;
	struct audio_codec_cfg codec_cfg = {
		.mclk_freq = DT_PROP(DT_NODELABEL(es8311), mclk_frequency_hz),
		.dai_type = AUDIO_DAI_TYPE_I2S,
		.dai_cfg = dai,
		.dai_route = AUDIO_ROUTE_PLAYBACK,
	};
	if (audio_codec_configure(codec, &codec_cfg)) {
		LOG_WRN("codec configure failed");
		return -EIO;
	}
	audio_codec_start(codec, AUDIO_DAI_DIR_TX);

	struct i2s_config i2s_cfg = {
		.word_size = WORD_SIZE,
		.channels = NUM_CHANNELS,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = SAMPLE_RATE,
		.mem_slab = &audio_slab,
		.block_size = BLOCK_BYTES,
		.timeout = 2000,
	};
	if (i2s_configure(i2s_dev, I2S_DIR_BOTH, &i2s_cfg)) {
		LOG_WRN("i2s configure failed");
		return -EIO;
	}
	bsp_i2s_duplex_fix();

	for (int i = 0; i < 2; i++) {
		void *pre;
		if (k_mem_slab_alloc(&audio_slab, &pre, K_MSEC(50)) == 0) {
			memset(pre, 0, BLOCK_BYTES);
			i2s_write(i2s_dev, pre, BLOCK_BYTES);
		}
	}
	if (i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_START)) {
		LOG_WRN("i2s start failed");
		return -EIO;
	}
	ok = true;
	LOG_INF("audio: ES8311 TX @%u Hz", SAMPLE_RATE);
	return 0;
}

bool audio_ready(void)
{
	return ok;
}

/* queue up to NUM_BLOCKS of PCM; drops silently if the pipe is full —
 * a demod blip is best-effort audio, never worth stalling the SDR loop */
void audio_play(const int16_t *pcm, uint32_t frames)
{
	uint32_t done = 0;

	if (!ok) {
		return;
	}
	while (done < frames) {
		void *buf;
		uint32_t n = frames - done;
		int16_t *dst;

		if (n > BLOCK_FRAMES) {
			n = BLOCK_FRAMES;
		}
		if (k_mem_slab_alloc(&audio_slab, &buf, K_NO_WAIT)) {
			return;
		}
		dst = (int16_t *)buf;
		for (uint32_t i = 0; i < n; i++) {
			dst[i * 2] = dst[i * 2 + 1] = pcm[done + i];
		}
		if (n < BLOCK_FRAMES) {
			memset(dst + n * 2, 0, (BLOCK_FRAMES - n) * 4);
		}
		if (i2s_write(i2s_dev, buf, BLOCK_BYTES)) {
			k_mem_slab_free(&audio_slab, buf);
			return;
		}
		done += n;
	}
}

#else

int audio_init(void) { return -ENODEV; }
bool audio_ready(void) { return false; }
void audio_play(const int16_t *pcm, uint32_t frames)
{
	(void)pcm; (void)frames;
}

#endif
