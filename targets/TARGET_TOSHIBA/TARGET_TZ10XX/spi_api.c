/* mbed Microcontroller Library
 * Copyright 2017, Cerevo Inc. 
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "mbed_assert.h"
#include <math.h>

#include "spi_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "mbed_error.h"
#include "PeripheralPins.h"
#include "pmu.h"

#include "SPI_TZ10xx.h"
/* Number of SPI channel in TZ10xx */
#define TZ10XX_SPI_NUM   (4)
/* TZ10xx SPI driver object */
extern TZ10XX_DRIVER_SPI Driver_SPI0;
extern TZ10XX_DRIVER_SPI Driver_SPI1;
extern TZ10XX_DRIVER_SPI Driver_SPI2;
extern TZ10XX_DRIVER_SPI Driver_SPI3;
/* Initialized flag for each SPI module */
static bool spi_drv_init[TZ10XX_SPI_NUM];

/** Initialize the SPI peripheral module.
 *  @remarks
 *    + Configures the pins used by SPI, sets a default format and frequency,
 *    and enables the peripheral module.
 *    + Use CMSIS SPI Driver:
 *      - SPI_Initialize
 *      - SPI_PowerControl
 *      - SPI_SlaveSelect
 *
 *  @param  obj         The SPI object to initialize
 *  @param  mosi        The pin to use for MOSI
 *  @param  miso        The pin to use for MISO
 *  @param  sclk        The pin to use for SCLK
 *  @param  ssel        The pin to use for SSEL
 *
 *  @returns
 *    NONE.
 */
void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel)
{
    ARM_SPI_STATUS status;

    /* Determine the SPI to use */
    SPIName spi_mosi = (SPIName)pinmap_peripheral(mosi, PinMap_SPI_MOSI);
    SPIName spi_miso = (SPIName)pinmap_peripheral(miso, PinMap_SPI_MISO);
    SPIName spi_sclk = (SPIName)pinmap_peripheral(sclk, PinMap_SPI_SCLK);
    SPIName spi_ssel = (SPIName)pinmap_peripheral(ssel, PinMap_SPI_SCSN);
    SPIName spi_data = (SPIName)pinmap_merge(spi_mosi, spi_miso);
    SPIName spi_cntl = (SPIName)pinmap_merge(spi_sclk, spi_ssel);
    obj->module = (SPIName)pinmap_merge(spi_data, spi_cntl);

    MBED_ASSERT((int)obj->module != NC);

    switch ((int)obj->module) {
        case SPIM_0:
            obj->driver = &Driver_SPI0;
            break;
        case SPIM_1:
            obj->driver = &Driver_SPI1;
            break;
        case SPIM_2:
            obj->driver = &Driver_SPI2;
            break;
        case SPIM_3:
            obj->driver = &Driver_SPI3;
            break;
        default:
            obj->driver = NULL;
            obj->module = (SPIName)NC;
            error("Cannot found SPI module corresponding with input pins.");
            break;
    }

    MBED_ASSERT (obj->driver != NULL);

    /* Setup PMU */
    PmuInit();

    /* Pin out the spi pins */
    pinmap_pinout(mosi, PinMap_SPI_MOSI);
    pin_mode(mosi, PullDown);
    pinmap_pinout(miso, PinMap_SPI_MISO);
    pin_mode(miso, PullDown);
    pin_mode(miso, InputActive);

    pinmap_pinout(sclk, PinMap_SPI_SCLK);
    if (ssel != NC) {
        pinmap_pinout(ssel, PinMap_SPI_SCSN);
    }

    /* If the module initialized, TZ10XX SPI driver will not invoke */
    if(spi_drv_init[obj->module]) {
        return;
    }
    /* Enable power and clocking */
    status = obj->driver->Initialize(NULL);
    MBED_ASSERT(ARM_SPI_OK == status);
    status = obj->driver->PowerControl(ARM_POWER_FULL);
    MBED_ASSERT(ARM_SPI_OK == status);

    /*
     * SPI of theTZ1000 series cannot control the SlaveSelect signal with software.
     *  This function is implemented to conform to the CMSIS driver interface.
     *  The SlaveSelect signal is enabled at calling of a data transfer function and is disabled upon
     *  completion of the data transfer function.
     */
    if (ssel != NC) {
        status = obj->driver->SlaveSelect(ARM_SPI_SS_ACTIVE);
        MBED_ASSERT(ARM_SPI_OK == status);
    }
    spi_drv_init[obj->module] = true;
}

/** De-Initialize a SPI peripheral.
 *  @remarks
 *    + Return the pins owned by the SPI object to their reset state
 *      - Disable the SPI peripheral
 *      - Disable the SPI clock
 *    + Use CMSIS SPI Driver:
 *      - SPI_Uninitialize
 *
 *  @param  obj         The SPI object to de-initialize
 *
 *  @returns
 *    NONE.
 */
void spi_free(spi_t *obj)
{
    if(!spi_drv_init[obj->module]) {
        return;
    }
    ARM_SPI_STATUS status;
    status = obj->driver->Uninitialize();
    MBED_ASSERT(ARM_SPI_OK == status);
    obj->driver = NULL;
    obj->module = (SPIName)NC;
}

/* Slave mode is not supported */

/** Set the number of bits per frame, configure clock polarity and phase,
 * shift order and master/slave mode.
 *  @remarks
 *    + Use CMSIS SPI Driver:
 *      - SPI_Configure
 *      - SPI_FrameSize
 *
 *  @param  obj         The SPI object to configure
 *  @param  bits        The number of bits per frame
 *  @param  mode        The SPI mode (clock polarity, phase, and shift direction)
 *  @param  slave       Zero for master mode or non-zero for slave mode
 *
 *  @returns
 *    NONE.
 */
void spi_format(spi_t *obj, int bits, int mode, int slave)
{
    ARM_SPI_STATUS status;
    if(!spi_drv_init[obj->module]) {
        return;
    }

    /* Configure the SPI mode (clock polarity, phase, and shift direction) */
    status = obj->driver->Configure((ARM_SPI_FRAME_FORMAT)mode, ARM_SPI_MSB_LSB);
    MBED_ASSERT(ARM_SPI_OK == status);

    /* The number of bits per frame */
    status = obj->driver->FrameSize((uint32_t)(bits - 1));
    MBED_ASSERT(ARM_SPI_OK == status);
}

/** Configures the SPI peripheral's baud rate.
 *  @remarks
 *    + Actual frequency may differ from the desired frequency due to
 *    available dividers and bus clock.
 *    + Use CMSIS SPI Driver:
 *      - SPI_BusSpeed
 *
 *  @param  obj         The SPI object to configure
 *  @param  hz          The baud rate in Hz (bps)
 *
 *  @returns
 *    NONE.
 */
void spi_frequency(spi_t *obj, int hz)
{
    if(!spi_drv_init[obj->module]) {
        return;
    }
    obj->driver->BusSpeed((uint32_t)hz);
}

/** Write a byte out in master mode and receive a value.
 *  @remarks
 *    + Synchronous SPI
 *    + Use CMSIS SPI Driver:
 *      - SPI_TransferFrame
 *
 *  @param  obj         The SPI object to use for send
 *  @param  value       The value to send
 *
 *  @returns
 *    One data frame.
 */
int spi_master_write(spi_t *obj, int value)
{
    if(!spi_drv_init[obj->module]) {
        return 0;
    }

    return (int)obj->driver->TransferFrame((uint16_t)value);
}

/** Get number of SPI modules .
 *
 *  @param  obj         The SPI object to get module number
 *
 *  @returns
 *    SPI module number.
 */
uint8_t spi_get_module(spi_t *obj)
{
    /* Return SPI module number */
    return (uint8_t)(obj->module);
}
