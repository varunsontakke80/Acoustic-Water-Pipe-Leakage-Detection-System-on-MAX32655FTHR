/**
 * @file    leak_detector.h
 * @brief   Acoustic Pipe Leakage Detection DSP Engine for MAX32655FTHR
 */

#ifndef LEAK_DETECTOR_H_
#define LEAK_DETECTOR_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Leak Detection State */
typedef enum {
    LEAK_STATE_NORMAL = 0,         /* Pipe is intact; normal quiet baseline */
    LEAK_STATE_TRANSIENT,          /* High transient noise ignored (knock/tap/wind) */
    LEAK_STATE_SUSPECTED,          /* Persistent elevated acoustic noise detected */
    LEAK_STATE_ALARM               /* Confirmed continuous acoustic leak vibration */
} leak_state_t;

/* Real-Time Acoustic & Leak Telemetry Metrics */
typedef struct {
    uint32_t      rms;             /* Filtered RMS acoustic amplitude */
    uint32_t      peak;            /* Peak absolute sample amplitude */
    float         crest_factor;    /* Peak / RMS ratio (identifies transient vs continuous) */
    uint32_t      zcr;             /* Zero-crossing rate per frame */
    uint32_t      baseline_rms;    /* Calibrated quiet baseline acoustic floor */
    uint32_t      threshold_rms;   /* Current leak detection threshold */
    leak_state_t  state;           /* Current detection state */
    bool          leak_detected;   /* True if leak alarm is active */
    uint8_t       confidence_pct;  /* Leak confidence level (0 - 100%) */
    uint8_t       severity_pct;    /* Estimated leak severity (0 - 100%) */
    uint32_t      persistence_cnt; /* Consecutive frames above threshold */
} leak_metrics_t;

/**
 * @brief  Initialize the acoustic pipe leak detection DSP engine.
 */
void LeakDetector_Init(void);

/**
 * @brief  Process a frame of raw 16-bit PCM audio samples.
 * @param  samples     Pointer to raw audio buffer from microphone.
 * @param  num_samples Number of samples in the frame.
 * @param  out_metrics Pointer to metrics structure to receive analysis results.
 * @return Current leak state.
 */
leak_state_t LeakDetector_Process(const int16_t *samples, uint32_t num_samples, leak_metrics_t *out_metrics);

/**
 * @brief  Trigger baseline calibration mode for specified number of frames.
 * @param  frames Number of frames to average (e.g., 20 frames = ~20 seconds at 1 Hz).
 */
void LeakDetector_StartCalibration(uint32_t frames);

/**
 * @brief  Check if calibration is currently active.
 */
bool LeakDetector_IsCalibrating(void);

/**
 * @brief  Manually set the RMS leak threshold.
 * @param  threshold_rms New threshold value.
 */
void LeakDetector_SetThreshold(uint32_t threshold_rms);

/**
 * @brief  Reset any latched leak alarm back to normal.
 */
void LeakDetector_ResetAlarm(void);

/**
 * @brief  Get the latest computed metrics.
 */
void LeakDetector_GetMetrics(leak_metrics_t *out_metrics);

#ifdef __cplusplus
}
#endif

#endif /* LEAK_DETECTOR_H_ */

