/**
 * @file    max9867.h
 * @brief   MAX9867 Audio Codec Driver Header
 */

#ifndef MAX9867_H_
#define MAX9867_H_

#include <i2c_regs.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX9867_INPUT_INT_MIC    0  /* Internal Digital MEMS Microphone */
#define MAX9867_INPUT_EXT_LINEIN 1  /* External Microphone plugged into 3.5mm Line-In */

int max9867_init(mxc_i2c_regs_t *i2c_inst, int mclk, int lrclk);
int max9867_set_input(int source);
int max9867_shutdown(void);
int max9867_status(void);

#ifdef __cplusplus
}
#endif

#endif /* MAX9867_H_ */
