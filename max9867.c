/**
 * @file    max9867.c
 * @brief   MAX9867 Audio Codec Driver with Dual Input Support (Internal Mic & External Line-In)
 */

#include "max9867.h"

#include <i2c.h>
#include <i2c_regs.h>
#include <mxc_device.h>
#include <mxc_delay.h>
#include <stddef.h>
#include <stdint.h>

#define I2C_FREQ 100000

#define MAX9867_ADDR 0x18
#define MAX9867_REVID 0x42

// Register addresses
#define MAX9867_00_STATUS            0x00
#define MAX9867_01_JACKSENSE         0x01
#define MAX9867_02_AUX_HIGH          0x02
#define MAX9867_03_AUX_LOW           0x03
#define MAX9867_04_INT_EN            0x04
#define MAX9867_05_SYS_CLK           0x05
#define MAX9867_06_CLK_HIGH          0x06
#define MAX9867_07_CLK_LOW           0x07
#define MAX9867_08_DAI_FORMAT        0x08
#define MAX9867_09_DAI_CLOCK         0x09
#define MAX9867_0A_DIG_FILTER        0x0A
#define MAX9867_0B_SIDETONE          0x0B
#define MAX9867_0C_LVL_DAC           0x0C
#define MAX9867_0D_LVL_ADC           0x0D
#define MAX9867_0E_LVL_LINE_IN_LEFT  0x0E
#define MAX9867_0F_LVL_LINE_IN_RIGHT 0x0F
#define MAX9867_10_VOL_LEFT          0x10
#define MAX9867_11_VOL_RIGHT         0x11
#define MAX9867_12_MIC_GAIN_LEFT     0x12
#define MAX9867_13_MIC_GAIN_RIGHT    0x13
#define MAX9867_14_ADC_INPUT         0x14
#define MAX9867_15_MIC               0x15
#define MAX9867_16_MODE              0x16
#define MAX9867_17_PWR_SYS           0x17
#define MAX9867_FF_REV_ID            0xFF

#define MAX9867_ICLD                 (1 << 7)
#define MAX9867_SDODLY               (1 << 2)
#define MAX9867_PSCLK_POS            4
#define MAX9867_PLL                  (1 << 7)
#define MAX9867_NI_UPPER_48KHZ       0x60
#define MAX9867_NI_LOWER_48KHZ       0x00
#define MAX9867_MAS                  (1 << 7)
#define MAX9867_WCI                  (1 << 6)
#define MAX9867_BCI                  (1 << 5)
#define MAX9867_DLY                  (1 << 4)
#define MAX9867_HIZOFF               (1 << 3)
#define MAX9867_TDM                  (1 << 2)
#define MAX9867_BSEL_PCLK_DIV8       0x06
#define MAX9867_AVL_POS              4
#define MAX9867_AVR_POS              0
#define MAX9867_LI_MUTE              (1 << 6)
#define MAX9867_LI_GAIN_POS          0
#define MAX9867_VOL_POS              0
#define MAX9867_MXINL_POS            6
#define MAX9867_MXINR_POS            4
#define MAX9867_MXIN_DIS             0
#define MAX9867_MXIN_ANALOG_MIC      1
#define MAX9867_MXIN_LINE            2
#define MAX9867_MICCLK_POS           6
#define MAX9867_DIGMICL_POS          5
#define MAX9867_DIGMICR_POS          4
#define MAX9867_HPMODE_POS           0
#define MAX9867_STEREO_SE_CLICKLESS  4
#define MAX9867_MONO_SE_CLICKLESS    5
#define MAX9867_SHDN                 (1 << 7)
#define MAX9867_LNLEN                (1 << 6)
#define MAX9867_LNREN                (1 << 5)
#define MAX9867_DALEN                (1 << 3)
#define MAX9867_DAREN                (1 << 2)
#define MAX9867_ADLEN                (1 << 1)
#define MAX9867_ADREN                (1 << 0)

#define MHZ(N)                       (N * 1000000)

#define MXC_I2C_ERROR \
    (MXC_F_I2C_INTFL0_ARB_ERR | MXC_F_I2C_INTFL0_TO_ERR | MXC_F_I2C_INTFL0_ADDR_NACK_ERR | \
     MXC_F_I2C_INTFL0_DATA_ERR | MXC_F_I2C_INTFL0_DNR_ERR | MXC_F_I2C_INTFL0_START_ERR |   \
     MXC_F_I2C_INTFL0_STOP_ERR)

#define I2C_TO_COUNT 50000

static mxc_i2c_regs_t *s_i2c = NULL;

static inline void i2c_flush(void)
{
    s_i2c->intfl0 = s_i2c->intfl0;
    s_i2c->intfl1 = s_i2c->intfl1;
    s_i2c->txctrl0 |= MXC_F_I2C_TXCTRL0_FLUSH;
    uint32_t to = 10000;
    while ((s_i2c->txctrl0 & MXC_F_I2C_TXCTRL0_FLUSH) && --to) {}
    s_i2c->rxctrl0 |= MXC_F_I2C_RXCTRL0_FLUSH;
    to = 10000;
    while ((s_i2c->rxctrl0 & MXC_F_I2C_RXCTRL0_FLUSH) && --to) {}
}

static int reg_write(uint8_t reg, uint8_t val)
{
    if (s_i2c == NULL) return E_UNINITIALIZED;

    i2c_flush();

    s_i2c->fifo = (MAX9867_ADDR << 1); // 0x30 write
    s_i2c->mstctrl |= MXC_F_I2C_MSTCTRL_START;
    s_i2c->fifo = reg;
    s_i2c->fifo = val;
    s_i2c->mstctrl |= MXC_F_I2C_MSTCTRL_STOP;

    uint32_t timeout = I2C_TO_COUNT;
    while (!(s_i2c->intfl0 & MXC_F_I2C_INTFL0_STOP)) {
        if (--timeout == 0) return -201;
    }
    s_i2c->intfl0 = MXC_F_I2C_INTFL0_STOP;

    return (s_i2c->intfl0 & MXC_I2C_ERROR) ? -202 : E_NO_ERROR;
}

static int reg_read(uint8_t reg, uint8_t *dest)
{
    if (s_i2c == NULL || dest == NULL) return E_UNINITIALIZED;

    i2c_flush();
    MXC_I2C_SetRXThreshold(s_i2c, 1);

    s_i2c->fifo = (MAX9867_ADDR << 1); // 0x30 write
    s_i2c->mstctrl = MXC_F_I2C_MSTCTRL_START;
    s_i2c->fifo = reg;

    s_i2c->rxctrl1 = 1;
    s_i2c->mstctrl = MXC_F_I2C_MSTCTRL_RESTART;
    uint32_t timeout = I2C_TO_COUNT;
    while (s_i2c->mstctrl & MXC_F_I2C_MSTCTRL_RESTART) {
        if (--timeout == 0) return -101;
    }
    s_i2c->fifo = (MAX9867_ADDR << 1) | 0x01; // 0x31 read

    timeout = I2C_TO_COUNT;
    while ((MXC_I2C_GetRXFIFOAvailable(s_i2c) == 0) &&
           !(s_i2c->intfl0 & (MXC_F_I2C_INTFL0_RX_THD | MXC_F_I2C_INTFL0_DONE))) {
        if (--timeout == 0) return -102;
    }
    *dest = (uint8_t)(s_i2c->fifo & 0xFF);
    s_i2c->intfl0 = MXC_F_I2C_INTFL0_RX_THD | MXC_F_I2C_INTFL0_DONE;

    s_i2c->mstctrl = MXC_F_I2C_MSTCTRL_STOP;
    timeout = I2C_TO_COUNT;
    while (!(s_i2c->intfl0 & MXC_F_I2C_INTFL0_STOP)) {
        if (--timeout == 0) return -103;
    }
    s_i2c->intfl0 = MXC_F_I2C_INTFL0_STOP;

    return (s_i2c->intfl0 & MXC_I2C_ERROR) ? -104 : E_NO_ERROR;
}

int max9867_status(void)
{
    uint8_t status = 0;
    int err = reg_read(MAX9867_00_STATUS, &status);
    if (err != E_NO_ERROR) {
        return err;
    }
    return status;
}

int max9867_set_input(int source)
{
    int err = E_NO_ERROR;

    if (source == MAX9867_INPUT_EXT_LINEIN) {
        /* ===== CONFIGURE EXTERNAL MICROPHONE / LINE-IN (3.5mm JACK) ===== */
        /* 1. Put codec in shutdown to safely update analog routing */
        reg_write(MAX9867_17_PWR_SYS, 0x00);

        /* 2. Disable digital microphone */
        reg_write(MAX9867_15_MIC, 0x00);

        /* 3. Route Line-In and Mic-In to Left and Right ADC */
        /* 0xF0 = ADC_IN_LINE_IN_ANALOG_MIC on Left (bits 7:6) and Right (bits 5:4) */
        reg_write(MAX9867_14_ADC_INPUT, 0xF0);

        /* 4. Set Line-In preamplifier gain to +24 dB (maximum sensitivity), unmute */
        /* Bits 3:0 = 0x0F (+24 dB gain), Bit 6 = 0 (unmuted) */
        reg_write(MAX9867_0E_LVL_LINE_IN_LEFT, 0x0F);
        reg_write(MAX9867_0F_LVL_LINE_IN_RIGHT, 0x0F);

        /* 5. Set Analog Mic Preamp to +30 dB and PGA to +20 dB for external mic elements */
        /* Bit 7:6 = 3 (+30dB), Bits 4:0 = 20 (+20dB PGA) -> 0xD4 */
        reg_write(MAX9867_12_MIC_GAIN_LEFT, 0xD4);
        reg_write(MAX9867_13_MIC_GAIN_RIGHT, 0xD4);

        /* 6. Set ADC digital volume to +3 dB */
        reg_write(MAX9867_0D_LVL_ADC, (3 << MAX9867_AVL_POS) | (3 << MAX9867_AVR_POS));

        /* 7. Power up System, Left Line-In, Right Line-In, Left ADC, Right ADC */
        /* SHDN(0x80) | LNLEN(0x40) | LNREN(0x20) | ADLEN(0x02) | ADREN(0x01) = 0xE3 */
        err = reg_write(MAX9867_17_PWR_SYS, MAX9867_SHDN | MAX9867_LNLEN | MAX9867_LNREN | MAX9867_ADLEN | MAX9867_ADREN);
    } else {
        /* ===== CONFIGURE INTERNAL DIGITAL MEMS MICROPHONE ===== */
        /* 1. Put codec in shutdown to reconfigure */
        reg_write(MAX9867_17_PWR_SYS, 0x00);

        /* 2. Disconnect analog inputs from ADC */
        reg_write(MAX9867_14_ADC_INPUT, 0x00);

        /* 3. Mute analog line inputs */
        reg_write(MAX9867_0E_LVL_LINE_IN_LEFT, MAX9867_LI_MUTE);
        reg_write(MAX9867_0F_LVL_LINE_IN_RIGHT, MAX9867_LI_MUTE);

        /* 4. Enable both Left & Right Digital Microphone inputs */
        /* DIGMICL (bit 5) | DIGMICR (bit 4) = 0x30 */
        reg_write(MAX9867_15_MIC, (1 << MAX9867_DIGMICL_POS) | (1 << MAX9867_DIGMICR_POS));

        /* 5. Set ADC digital volume to +3 dB */
        reg_write(MAX9867_0D_LVL_ADC, (3 << MAX9867_AVL_POS) | (3 << MAX9867_AVR_POS));

        /* 6. Power up System, Left ADC, Right ADC */
        /* SHDN(0x80) | ADLEN(0x02) | ADREN(0x01) = 0x83 */
        err = reg_write(MAX9867_17_PWR_SYS, MAX9867_SHDN | MAX9867_ADLEN | MAX9867_ADREN);
    }

    return err;
}

int max9867_init(mxc_i2c_regs_t *i2c_inst, int mclk, int lrclk)
{
    int err;
    uint8_t rev_id = 0;
    int psclk;

    s_i2c = i2c_inst;

    err = MXC_I2C_Init(i2c_inst, TRUE, 0);
    if (err != E_NO_ERROR) {
        return err;
    }
    MXC_I2C_SetFrequency(i2c_inst, I2C_FREQ);

    /* Probe for MAX9867 revision register (0xFF == 0x42) */
    err = reg_read(MAX9867_FF_REV_ID, &rev_id);
    extern volatile uint8_t s_codec_rev_id;
    s_codec_rev_id = rev_id;
    if (err != E_NO_ERROR) {
        return err;
    }
    if (rev_id != MAX9867_REVID) {
        return E_NOT_SUPPORTED;
    }

    /* Shutdown during initial configuration */
    reg_write(MAX9867_17_PWR_SYS, 0x00);

    /* Clear all registers to POR state */
    for (int i = MAX9867_04_INT_EN; i < MAX9867_17_PWR_SYS; i++) {
        reg_write(i, 0x00);
    }

    /* Select MCLK prescaler (PCLK between 10 MHz and 20 MHz) */
    if ((mclk < MHZ(10)) || (mclk > MHZ(60))) {
        return E_INVALID;
    } else if (mclk < MHZ(20)) {
        psclk = 0x1;
    } else if (mclk < MHZ(40)) {
        psclk = 0x2;
    } else {
        psclk = 0x3;
    }

    reg_write(MAX9867_05_SYS_CLK, psclk << MAX9867_PSCLK_POS);

    /* Configure 48 kHz sampling in master mode */
    reg_write(MAX9867_06_CLK_HIGH, MAX9867_NI_UPPER_48KHZ);
    reg_write(MAX9867_07_CLK_LOW, MAX9867_NI_LOWER_48KHZ);
    reg_write(MAX9867_09_DAI_CLOCK, MAX9867_BSEL_PCLK_DIV8);

    /* I2S format, master mode, delay 1 */
    reg_write(MAX9867_08_DAI_FORMAT, MAX9867_MAS | MAX9867_DLY | MAX9867_HIZOFF);

    /* Digital filter */
    reg_write(MAX9867_0A_DIG_FILTER, 0xA2);

    /* Headphone mode clickless */
    reg_write(MAX9867_16_MODE, MAX9867_STEREO_SE_CLICKLESS << MAX9867_HPMODE_POS);

    /* Volume settings */
    reg_write(MAX9867_10_VOL_LEFT, 0x04 << MAX9867_VOL_POS);
    reg_write(MAX9867_11_VOL_RIGHT, 0x04 << MAX9867_VOL_POS);

    /* Set default input to Internal Digital MEMS Microphone */
    return max9867_set_input(MAX9867_INPUT_INT_MIC);
}

int max9867_shutdown(void)
{
    reg_write(MAX9867_08_DAI_FORMAT, 0x00);
    return reg_write(MAX9867_17_PWR_SYS, 0x00);
}
