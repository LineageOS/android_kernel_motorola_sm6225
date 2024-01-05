//
// Description: Spi Driver Interface
//
// Created by Pascal Frederic St-Laurent
// Copyright (c) 2019 Boreas Technologies All rights reserved.
//

#ifndef DKCORE_SPI_H
#define DKCORE_SPI_H

#include <linux/types.h>

typedef struct _spi Spi;
typedef struct _spi Spi_t;


typedef int32_t (*SpiSend)(const Spi *spi, const void *data, size_t num);
typedef int32_t (*SpiRead)(const Spi *spi, void *data, size_t num);
typedef int32_t (*SpiTransfer)(const Spi *spi, const void *data_out, void *data_in, size_t num);
typedef int32_t (*SpiChipSelect)(const Spi *spi, bool assertCs);

struct _spi
{
    SpiSend send;
    SpiRead read;
    SpiTransfer transfer;
    SpiChipSelect cs;
};

bool isSameSpiPeripheral(const Spi* const spiA, const Spi* const spiB);

#endif //DKCORE_SPI_H
