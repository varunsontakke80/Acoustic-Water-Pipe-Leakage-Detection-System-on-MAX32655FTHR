/**
 * @file    leak_detector.c
 * @brief   Acoustic Pipe Leakage Detection DSP Engine for MAX32655FTHR
 */

#include "leak_detector.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Algorithm Parameters */
#define DEFAULT_BASELINE_RMS       120
#define DEFAULT_THRESHOLD_OFFSET   300
#define PERSISTENCE_CONFIRM_FRAMES 3       /* 3 consecutive elevated frames confirm leak */
#define MAX_CREST_FACTOR_FOR_LEAK  5.5f    /* Turbulent leak hiss has low crest factor */
#define MIN_CREST_FACTOR_FOR_LEAK  1.2f
#define HPF_ALPHA                  0.95f   /* DC-blocker filter coefficient */

static leak_metrics_t s_metrics;
static float s_hpf_x_prev = 0.0f;
static float s_hpf_y_prev = 0.0f;

/* Calibration state */
static bool s_calibrating = false;
static uint32_t s_calib_total_frames = 0;
static uint32_t s_calib_frame_cnt = 0;
static uint64_t s_calib_sum_rms = 0;

void LeakDetector_Init(void)
{
    memset(&s_metrics, 0, sizeof(s_metrics));
    s_metrics.baseline_rms    = DEFAULT_BASELINE_RMS;
    s_metrics.threshold_rms   = DEFAULT_BASELINE_RMS + DEFAULT_THRESHOLD_OFFSET;
    s_metrics.state           = LEAK_STATE_NORMAL;
    s_metrics.leak_detected   = false;
    s_metrics.confidence_pct  = 0;
    s_metrics.severity_pct    = 0;
    s_metrics.persistence_cnt = 0;

    s_hpf_x_prev = 0.0f;
    s_hpf_y_prev = 0.0f;
    s_calibrating = false;
}

void LeakDetector_StartCalibration(uint32_t frames)
{
    if (frames < 5) {
        frames = 5;
    }
    s_calib_total_frames = frames;
    s_calib_frame_cnt = 0;
    s_calib_sum_rms = 0;
    s_calibrating = true;
    printf("[LEAK_DSP] Starting baseline noise floor calibration (%u frames)...\n", (unsigned int)frames);
}

bool LeakDetector_IsCalibrating(void)
{
    return s_calibrating;
}

void LeakDetector_SetThreshold(uint32_t threshold_rms)
{
    s_metrics.threshold_rms = threshold_rms;
    printf("[LEAK_DSP] Custom threshold set to: %u\n", (unsigned int)threshold_rms);
}

void LeakDetector_ResetAlarm(void)
{
    s_metrics.state = LEAK_STATE_NORMAL;
    s_metrics.leak_detected = false;
    s_metrics.persistence_cnt = 0;
    s_metrics.confidence_pct = 0;
    s_metrics.severity_pct = 0;
    printf("[LEAK_DSP] Leak alarm latch reset.\n");
}

void LeakDetector_GetMetrics(leak_metrics_t *out_metrics)
{
    if (out_metrics != NULL) {
        memcpy(out_metrics, &s_metrics, sizeof(leak_metrics_t));
    }
}

leak_state_t LeakDetector_Process(const int16_t *samples, uint32_t num_samples, leak_metrics_t *out_metrics)
{
    if (samples == NULL || num_samples == 0) {
        return s_metrics.state;
    }

    uint64_t sum_squares = 0;
    uint32_t peak_val = 0;
    uint32_t zero_crossings = 0;
    float prev_filtered = 0.0f;

    /* 1. DC Blocker / High-pass filter & Feature extraction */
    for (uint32_t i = 0; i < num_samples; i++) {
        float x = (float)samples[i];
        /* 1st order DC blocker: y[n] = x[n] - x[n-1] + alpha * y[n-1] */
        float y = x - s_hpf_x_prev + (HPF_ALPHA * s_hpf_y_prev);
        s_hpf_x_prev = x;
        s_hpf_y_prev = y;

        float abs_y = fabsf(y);
        if (abs_y > (float)peak_val) {
            peak_val = (uint32_t)abs_y;
        }

        sum_squares += (uint64_t)(y * y);

        /* Zero crossing rate */
        if (i > 0) {
            if ((prev_filtered < 0.0f && y >= 0.0f) || (prev_filtered >= 0.0f && y < 0.0f)) {
                zero_crossings++;
            }
        }
        prev_filtered = y;
    }

    /* 2. Compute RMS & Crest Factor */
    uint32_t rms = (uint32_t)sqrtf((float)sum_squares / (float)num_samples);
    float crest_factor = (float)peak_val / ((float)rms + 1.0f);

    s_metrics.rms = rms;
    s_metrics.peak = peak_val;
    s_metrics.crest_factor = crest_factor;
    s_metrics.zcr = zero_crossings;

    /* 3. Handle Ongoing Calibration */
    if (s_calibrating) {
        s_calib_sum_rms += rms;
        s_calib_frame_cnt++;
        if (s_calib_frame_cnt >= s_calib_total_frames) {
            s_metrics.baseline_rms = (uint32_t)(s_calib_sum_rms / s_calib_frame_cnt);
            if (s_metrics.baseline_rms < 50) {
                s_metrics.baseline_rms = 50; /* minimum safety floor */
            }
            /* Set threshold proportional to baseline noise */
            s_metrics.threshold_rms = (uint32_t)((float)s_metrics.baseline_rms * 2.2f) + DEFAULT_THRESHOLD_OFFSET;
            s_calibrating = false;
            printf("[LEAK_DSP] Calibration Complete! Baseline RMS: %u, Threshold RMS: %u\n",
                   (unsigned int)s_metrics.baseline_rms, (unsigned int)s_metrics.threshold_rms);
        }
        if (out_metrics != NULL) {
            memcpy(out_metrics, &s_metrics, sizeof(leak_metrics_t));
        }
        return s_metrics.state;
    }

    /* 4. Acoustic Pipe Leak Detection Classification */
    if (rms > s_metrics.threshold_rms) {
        /* Check crest factor: true continuous leak hiss has low/moderate crest factor */
        if (crest_factor > MAX_CREST_FACTOR_FOR_LEAK) {
            /* High peak relative to RMS = transient mechanical spike (tapping/banging/wind gust) */
            s_metrics.state = LEAK_STATE_TRANSIENT;
            /* Do not increment persistence on transient bursts */
            if (s_metrics.persistence_cnt > 0) {
                s_metrics.persistence_cnt--;
            }
        } else {
            /* Continuous stationary elevated acoustic energy detected */
            s_metrics.persistence_cnt++;
            if (s_metrics.persistence_cnt >= PERSISTENCE_CONFIRM_FRAMES) {
                s_metrics.state = LEAK_STATE_ALARM;
                s_metrics.leak_detected = true;
            } else {
                s_metrics.state = LEAK_STATE_SUSPECTED;
            }
        }
    } else {
        /* Acoustic energy is within normal baseline limits */
        if (s_metrics.persistence_cnt > 0) {
            s_metrics.persistence_cnt--;
        }
        if (s_metrics.persistence_cnt == 0) {
            s_metrics.state = LEAK_STATE_NORMAL;
            s_metrics.leak_detected = false;
        }
    }

    /* 5. Compute Severity & Confidence */
    if (s_metrics.leak_detected) {
        uint32_t delta = rms > s_metrics.threshold_rms ? (rms - s_metrics.threshold_rms) : 0;
        uint32_t sev = (delta * 100) / (s_metrics.threshold_rms + 1);
        if (sev > 100) {
            sev = 100;
        }
        s_metrics.severity_pct = (uint8_t)sev;

        /* Confidence rises with persistence */
        uint32_t conf = 70 + (s_metrics.persistence_cnt * 5);
        if (conf > 99) {
            conf = 99;
        }
        s_metrics.confidence_pct = (uint8_t)conf;
    } else if (s_metrics.state == LEAK_STATE_SUSPECTED) {
        s_metrics.severity_pct = 25;
        s_metrics.confidence_pct = 40;
    } else {
        s_metrics.severity_pct = 0;
        s_metrics.confidence_pct = 0;
    }

    if (out_metrics != NULL) {
        memcpy(out_metrics, &s_metrics, sizeof(leak_metrics_t));
    }

    return s_metrics.state;
}

