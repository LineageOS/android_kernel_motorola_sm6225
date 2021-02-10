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

typedef int32_t(*SpiSend)(Spi *spi, const void *data, uint32_t num);

typedef int32_t(*SpiTransfer)(Spi *spi, const void *data_out, void *data_in, uint32_t num);

struct _spi
{
    SpiSend send;
    SpiTransfer transfer;
};

#endif //DKCORE_SPI_H
