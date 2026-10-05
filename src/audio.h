#pragma once
#include <stdint.h>

int  audio_init(void);                       /* ES8311 + i2s0 TX setup   */
bool audio_ready(void);
void audio_play(const int16_t *pcm, uint32_t frames); /* stereo 48k s16 */
