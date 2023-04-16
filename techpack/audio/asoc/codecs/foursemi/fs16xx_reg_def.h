/* 
* Copyright (C) 2016 Fourier Semiconductor Inc. 
*
*/

#ifndef FS16XX_REG_DEF_H
#define FS16XX_REG_DEF_H

#include "fs16xx_regs.h"
#include "fs16xx.h"

/* I2S_CONTROL bits for sample rates */
#define FS16XX_I2SCTRL_RATE_08000 (0<<FS16XX_I2SCTRL_REG_I2SSR_POS)
#define FS16XX_I2SCTRL_RATE_16000 (3<<FS16XX_I2SCTRL_REG_I2SSR_POS)
#define FS16XX_I2SCTRL_RATE_44100 (7<<FS16XX_I2SCTRL_REG_I2SSR_POS)
#define FS16XX_I2SCTRL_RATE_48000 (8<<FS16XX_I2SCTRL_REG_I2SSR_POS)

const struct fs16xx_regs Fs16xx_reg_defaults[] = {
{
    .reg = 0x06,
    .value = 0xFF00, // volume to 0dB
},
{
    .reg = 0x0B,
    .value = 0xCA91,
},
{
    .reg = 0xD3,
    .value = 0x0100, // damp, key protected.
},
{
    .reg = 0x0B,
    .value = 0x0000,
},
{
    .reg = 0xC1,
    .value = 0x0260, // configure pll
},
{
    .reg = 0xC2,
    .value = 0x0100, // configure pll
},
{
    .reg = 0xC3,
    .value = 0x0004, // configure pll
},
{
    .reg = 0x04,
    .value = 0x881B, // DC/DC mode: Follow, enable boost
},
{
    .reg = 0xC0,
    .value = 0x5B80, // DC/DC mode: Follow, enable boost
},
{
    .reg = 0xC4,
    .value = 0x000F,
},
{
    .reg = 0xAE,
    .value = 0x0210,
},
{
    .reg = 0xB9,
    .value = 0xFFFF,
},
{
    .reg = 0x09,
    .value = 0x0009, // ClassD always on, power down
},
{
    .reg = 0xCD,
    .value = 0x2004,
},
{
    .reg = 0xa1,
    .value = 0x1C92,
},
};

#endif
