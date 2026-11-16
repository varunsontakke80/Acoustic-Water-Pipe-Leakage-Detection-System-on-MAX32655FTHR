/**
 * @file    audio_mic.h
 * @brief   MAX32655FTHR Audio Acquisition Driver (Internal Mic & External Line-In)
 */

#ifndef AUDIO_MIC_H_
#define AUDIO_MIC_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_SAMPLE_RATE       48000
#define AUDIO_FRAME_SIZE        512     /* 512 samples @ 48kHz = ~10.7 ms acquisition */

typedef enum {
    AUDIO_MIC_SOURCE_INTERNAL = 0,  /* Onboard Digital MEMS Microphone */
    AUDIO_MIC_SOURCE_EXTERNAL = 1   /* External Microphone plugged into 3.5mm Line-In */
} audio_mic_source_t;

/**
 * @brief  Initialize PMIC mic rail, MAX9867 codec via I2C1, and I2S RX with DMA.
 * @return 0 on success, negative error code on failure.
 */
int Audio_Mic_Init(void);

/**
 * @brief  Select active microphone source (Internal MEMS or External Line-In).
 * @param  source  AUDIO_MIC_SOURCE_INTERNAL or AUDIO_MIC_SOURCE_EXTERNAL.
 * @return 0 on success, error code on failure.
 */
int Audio_Mic_SetSource(audio_mic_source_t source);

/**
 * @brief  Get currently active microphone source.
 */
audio_mic_source_t Audio_Mic_GetSource(void);

/**
 * @brief  Get string label for active microphone source.
 */
const char* Audio_Mic_GetSourceName(void);

/**
 * @brief  Capture one frame of 16-bit PCM audio samples from the active mic.
 * @param  out_buf     Pointer to destination buffer (must hold at least num_samples int16_t).
 * @param  num_samples Number of samples to read (typically AUDIO_FRAME_SIZE).
 * @return 0 on success, negative error code on timeout/failure.
 */
int Audio_Mic_CaptureFrame(int16_t *out_buf, uint32_t num_samples);

/**
 * @brief  Perform an active acoustic sound check on the specified microphone.
 * @param  source   Microphone source to test.
 * @param  min_val  Outputs minimum sample value observed.
 * @param  max_val  Outputs maximum sample value observed.
 * @param  rms_val  Outputs calculated RMS energy.
 * @param  hw_active Outputs whether real hardware I2S DMA was captured.
 * @return 0 on success, error code on failure.
 */
int Audio_Mic_Test(audio_mic_source_t source, int16_t *min_val, int16_t *max_val, uint32_t *rms_val, bool *hw_active);

/**
 * @brief  Inject or clear simulated pipe leak acoustic signal for testing.
 * @param  inject  true to simulate high-pressure leak sound, false for baseline.
 */
void Audio_Mic_InjectLeakSignal(bool inject);

/**
 * @brief  Power off codec and microphone to save power.
 */
void Audio_Mic_Shutdown(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_MIC_H_ */
