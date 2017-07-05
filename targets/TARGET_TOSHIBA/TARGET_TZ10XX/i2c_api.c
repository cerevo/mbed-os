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
#include "i2c_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "pmu.h"

#include "mbed_error.h"
#include "PeripheralPins.h"

#include "Driver_I2C.h"
#include "PMU_TZ10xx.h"

#define GENERAL_CALL_ADDRESS 0x0
#define MASTER_HOLD_BUS 1
#define ISSUE_STOP      0

/* PMU Driver interface */
extern TZ10XX_DRIVER_PMU Driver_PMU;

/* I2C CMSIS interface */
extern ARM_DRIVER_I2C      Driver_I2C0;
extern ARM_DRIVER_I2C      Driver_I2C1;
extern ARM_DRIVER_I2C      Driver_I2C2;

/** Initialize the I2C protocol
 *
 *  @param  obj         The I2C object to initialize.
 *  @param  sda         The pin to use for SDA.
 *  @param  scl         The pin to use for SCL.
 *
 *  @returns
 *    NONE.
 */
void i2c_init(i2c_t *obj, PinName sda, PinName scl)
{
    /* determine the I2C to use */
    I2CName i2c_sda = (I2CName)pinmap_peripheral(sda, PinMap_I2C_SDA);
    I2CName i2c_scl = (I2CName)pinmap_peripheral(scl, PinMap_I2C_SCL);
    I2CName i2c_name = (I2CName)pinmap_merge(i2c_sda, i2c_scl);
    if ((int)i2c_name == NC) {
        error("I2C pinout mapping failed");
    }

    obj->index = i2c_name;
    switch (i2c_name) {
        case I2C_0:
            obj->i2c = &Driver_I2C0;
            break;
        case I2C_1:
            obj->i2c = &Driver_I2C1;
            break;
        case I2C_2:
            obj->i2c = &Driver_I2C2;
            break;
        default:
            error("I2C is not available");
            break;
    }

    /* Setup PMU */
    PmuInit();

    /* pinout the chosen I2C */
    pinmap_pinout(sda, PinMap_I2C_SDA);
    pin_mode(sda, PullUp);
    pin_mode(sda, InputActive);
    pinmap_pinout(scl, PinMap_I2C_SCL);
    pin_mode(scl, PullUp);
    pin_mode(scl, InputActive);

    /* I2C initialize */
    obj->i2c->Initialize(NULL);

    /* set default frequency: Standard Speed (100 kHz) */
    /* set to ARM_POWER_FULL before setting speed */
    obj->i2c->PowerControl(ARM_POWER_FULL);
    obj->i2c->BusSpeed(ARM_I2C_BUS_SPEED_STANDARD);
}

/** Set frequency for I2C object
 *
 *  @param  obj         The I2C object to de-initialize.
 *  @param  hz          The frequency (hz).
 *
 *  @returns
 *    NONE.
 */
void i2c_frequency(i2c_t *obj, int hz)
{
    MBED_ASSERT((hz == 100000) || (hz == 400000));
    if (hz == 100000) {
        obj->i2c->BusSpeed(ARM_I2C_BUS_SPEED_STANDARD);
    } else if (hz == 400000) {
        obj->i2c->BusSpeed(ARM_I2C_BUS_SPEED_FAST);
    }
}

/** Master send data
 *  @remarks
 *    Empty function, because START condition is implemented
 *    in i2c_read() and i2c_write()
 *
 *  @param  obj         The I2C object to issue START condition.
 *
 *  @returns
 *    int   value       : 0.
 */
inline int i2c_start(i2c_t *obj)
{
    return 0;
}

/** Master issues STOP condition on I2C bus
 *  @remarks
 *    Empty function, because STOP condition is implemented
 *    in i2c_read() and i2c_write()
 *
 *  @param  obj         The I2C object to issue STOP condition.
 *
 *  @returns
 *    int value         : 0.
 */
inline int i2c_stop(i2c_t *obj)
{
    return 0;
}

/** Master-receiver reads data from Slave-transmitter
 *
 *  @param  obj         The I2C object as master-receiver.
 *  @param  address     Address of slave-transmitter.
 *  @param  data        Read buffer.
 *  @param  length      Read buffer size.
 *  @param  stop        The STOP condition.
 *
 *  @returns
 *    Number of bytes read
 */
int i2c_read(i2c_t *obj, int address, char *data, int length, int stop)
{
    int xfer_pending = 0;
    if (stop) {
        /* A read complete with STOP condition */
        xfer_pending = ISSUE_STOP;
    } else {
        /* Master holds the bus */
        xfer_pending = MASTER_HOLD_BUS;
    }

    /* We need to shift address because TZ10xx need 7 bit address */
    return obj->i2c->ReceiveData((address >> 1), (uint8_t*)data, length, xfer_pending);
}

/** Master-transmitter writes data to Slave-receiver
 *
 *  @param  obj         The I2C object as master-transmitter.
 *  @param  address     Address of slave-receiver.
 *  @param  data        The buffer for writing.
 *  @param  length      Buffer size.
 *  @param  stop        The STOP condition.
 *
 *  @returns
 *    Number of written bytes
 */
int i2c_write(i2c_t *obj, int address, const char *data, int length, int stop)
{
    int xfer_pending = 0;
    if (stop) {
        /* A write complete with STOP condition */
        xfer_pending = ISSUE_STOP;
    } else {
        /* Master hold the bus */
        xfer_pending = MASTER_HOLD_BUS;
    }

    /* We need to shift address because TZ10xx need 7 bit address */
    return obj->i2c->SendData((address >> 1), (const uint8_t*)data, length, xfer_pending);
}

/** Reset and stop I2C transmit
 *
 *  @param  obj         The I2C object need to reset.
 *
 *  @returns
 *    NONE.
 */
void i2c_reset(i2c_t *obj)
{
    i2c_stop(obj);
}

/** Master-receiver reads a byte from Slave-transmitter.
 *
 *  @param  obj         The I2C object as master-receiver.
 *  @param  last        The flag for operation
 *    Value of “last” parameter indicates whether master continues reading data.
 *    If “last” equals TRUE, then master continues reading after this reading time.
 *    If “last” equals FALSE, then master stops reading after this reading time.
 *
 *  @returns
 *    Number of read bytes
 */
int i2c_byte_read(i2c_t *obj, int last)
{
    int xfer_pending = 0;
    uint8_t data = 0;

    if (last) {
        /* This is last byte, issue a STOP condition */
        xfer_pending = ISSUE_STOP;
    } else {
        /* This is not last byte, master doesn't issue STOP condition*/
        xfer_pending = MASTER_HOLD_BUS;
    }

    obj->i2c->ReceiveData(GENERAL_CALL_ADDRESS, &data, 1, xfer_pending);
    return (int)data;
}

/** Master-transmitter writes a byte to Slave-receiver.
 *
 *  @param  obj         The I2C object as master-receiver.
 *  @param  data        Write data buffer.
 *
 *  @returns
 *    Number of written bytes
 */
int i2c_byte_write(i2c_t *obj, int data)
{
    /* Master holds bus after byte write */
    if ( obj->i2c->SendData(GENERAL_CALL_ADDRESS, (const uint8_t*)&data, 1, MASTER_HOLD_BUS ) != 1) {
        return 0;
    }

    return 1;
}
