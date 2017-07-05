/**
 * @file MAG_TZ10xx.c
 * @brief a header file for TZ10xx MAG driver
 * @date $Date:: 2014-12-19 11:39:40 +0900 #$
 * @note
 */
/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#include "RTE_Components.h"
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "MAG_TZ10xx.h"

#ifdef MAG_ENABLE_INTERRUPT
#include "GPIO_TZ10xx.h"
#endif

#if (RTE_MAG)

#if (0==RTE_I2C2)
#error "TZ10XX Magnetometer Driver needs I2C2 device."
#endif

#if (defined MAG_ENABLE_INTERRUPT) && ((0==RTE_GPIO_24_ID) || (0==RTE_GPIO))
#error "TZ10XX Magnetometer Driver needs GPIO pin #24."
#endif

#define ARM_MAG_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,17)   /* driver version */
#define ARM_MAG_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,17)   /* driver version */

/* Sensor slave address */
#define SENSOR_ADDR     (0x0C)

/* Sensor register map */
#define WIA1                 (0x00)
#define WIA2                 (0x01)
#define INFO1                (0x02)
#define INFO2                (0x03)
#define ST1                  (0x10)
#define HXL                  (0x11)
#define HXH                  (0x12)
#define HYL                  (0x13)
#define HYH                  (0x14)
#define HZL                  (0x15)
#define HZH                  (0x16)
#define TMPS                 (0x17)
#define ST2                  (0x18)
#define CNTL1                (0x30)
#define CNTL2                (0x31)
#define CNTL3                (0x32)
#define ASAX                 (0x60)
#define ASAY                 (0x61)
#define ASAZ                 (0x62)

/* CNTL1 bit */
#define CNTL1_BIT_TEM        (0x80)
#define CNTL1_BIT_NSF        (0x60)

/* CNTL2 value */
#define CNTL2_MODE_PW_DOWN   (0x00)
#define CNTL2_MODE_SINGLE    (0x01)
#define CNTL2_MODE_CONT_10   (0x02)
#define CNTL2_MODE_CONT_20   (0x04)
#define CNTL2_MODE_CONT_50   (0x06)
#define CNTL2_MODE_CONT_100  (0x08)
#define CNTL2_MODE_SELFTEST  (0x10)
#define CNTL2_MODE_ROM       (0x1F)

/* ST1 value */
#define ST1_DRDY             (0x1)
#define ST1_DOR              (0x2)
/* ST2 value */
#define ST2_HOFL             (0x800)

#define UNKNOWN_MODE         (0xFF)

/* bitband alias for PMU */
#define BB_PSW_MAG_VDDCW    BITBAND_VALUE(&pmulv->PSW_HARDMACRO,2)
#define BB_PSW_MAG_VDDCS    BITBAND_VALUE(&pmulv->PSW_HARDMACRO,3)

/* GPIO pins assigned to Magnetmeter */
#define MAG_GPIO_INT_PIN (24)
#define MAG_IO_FUNC_GPIO  PMU_IO_FUNC_GPIO_24

extern ARM_DRIVER_I2C  Driver_I2C2;
#ifdef MAG_ENABLE_INTERRUPT
extern TZ10XX_DRIVER_GPIO Driver_GPIO;
#endif

static MAG_STATUS MAG_PowerControl(ARM_POWER_STATE state);
static MAG_STATUS MAG_ResetSensor(void);

typedef struct _MAG_INFO {
  bool              init;
  ARM_POWER_STATE   power;
  uint8_t           mode;
  uint8_t           speed;
  MAG_SignalEvent_t cb;
  uint8_t           adj[3];    /* adjustment value */
#if defined MAG_ENABLE_NSF | defined MAG_ENABLE_TEMPERATURE
  uint8_t           cntl1;
#endif
} MAG_INFO;

typedef struct _MAG_RESOURCE {
  ARM_DRIVER_I2C *i2c;
  MAG_INFO       *info;
#ifdef MAG_ENABLE_INTERRUPT
  TZ10XX_DRIVER_GPIO *gpio;
#endif
} MAG_RESOURCE;

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_MAG_API_VERSION,
  ARM_MAG_DRV_VERSION
};

static const MAG_CAPABILITIES Capabilities;
static MAG_INFO MAG0_INFO;
static MAG_RESOURCE MAG0_RESOURCE = {
  &Driver_I2C2,
  &MAG0_INFO,
#ifdef MAG_ENABLE_INTERRUPT
  &Driver_GPIO,
#endif
};

static void MAG_Wait(uint32_t usec)
{
  uint32_t i;
  uint32_t clk;

  clk = SystemCoreClock;
  /* 4 cycles for loop overhead */
  if (clk < (1000000 * 4)) {
    i = clk * usec / (1000000 * 4);
  } else {
    i = clk / (1000000 * 4) * usec;
  }
  do {
    __NOP();
  } while (--i);
}

static void MAG_PowerOnSensor(void)
{
  if (0 == BB_PSW_MAG_VDDCS) {
    Driver_PMU.EnableSensor(PMU_SENSOR_MAG, 1);
    /* wait 100us for power on reset done */
    MAG_Wait(100);
  }

#ifdef MAG_ENABLE_INTERRUPT
  Driver_PMU.StandbyInputBuffer(MAG_IO_FUNC_GPIO, 0);
#endif

}

static void MAG_PowerOffSensor(void)
{
  Driver_PMU.EnableSensor(PMU_SENSOR_MAG, 0);
  /* wait 100us for next power. */
  MAG_Wait(100);
  
#ifdef MAG_ENABLE_INTERRUPT
  Driver_PMU.StandbyInputBuffer(MAG_IO_FUNC_GPIO, 1);
#endif
}

static MAG_STATUS MAG_ReadSt1(uint8_t* st)
{
  MAG_RESOURCE   *res = &MAG0_RESOURCE;
  ARM_DRIVER_I2C *i2c;
  uint8_t         cmd;
  int             len;

  i2c = res->i2c;
  cmd = ST1;
  len = i2c->SendData(SENSOR_ADDR, &cmd, sizeof(cmd), true);
  if (len < (int)sizeof(cmd)) {
    return MAG_ERROR_I2C_BUS;
  }
  len = i2c->ReceiveData(SENSOR_ADDR, st, sizeof(*st), false);
  if (len < (int)sizeof(*st)) {
    return MAG_ERROR_I2C_BUS;
  }
  return MAG_OK;
}

static MAG_STATUS MAG_WriteCntlX(uint8_t cntl, uint8_t val)
{
  MAG_RESOURCE   *res = &MAG0_RESOURCE;
  ARM_DRIVER_I2C *i2c;
  uint8_t         cmd[2];
  int             len;
  
  cmd[0] = cntl;
  cmd[1] = val;
  i2c = res->i2c;
  len = i2c->SendData(SENSOR_ADDR, cmd, sizeof(cmd), false);
  if (len < (int)sizeof(cmd)) {
    return MAG_ERROR_I2C_BUS;
  }
  return MAG_OK;
}

#if defined MAG_ENABLE_NSF | defined MAG_ENABLE_TEMPERATURE
static MAG_STATUS MAG_WriteCntl1(uint8_t val)
{
  return MAG_WriteCntlX(CNTL1, val);
}
#endif

static MAG_STATUS MAG_WriteCntl2(uint8_t mode)
{
  return MAG_WriteCntlX(CNTL2, mode);
}

static MAG_STATUS MAG_ResetSensor(void)
{
  MAG_STATUS  err;
  err = MAG_WriteCntlX(CNTL3, 0x01);
  if( err != MAG_OK ){
    return err;
  }
  else{
    /* wait until done */
    MAG_Wait(100);
    return MAG_OK;
  }
}

static void MAG_SensorModeUpdate(void)
{
  MAG_RESOURCE   *res = &MAG0_RESOURCE;
  MAG_INFO       *info;
  ARM_DRIVER_I2C *i2c;
  uint8_t         cmd, mode;
  int32_t         speed;
  int             len;

  i2c   = res->i2c;
  info  = res->info;
  speed = info->speed;
  if (speed == 0) {
    if (ARM_I2C_OK == i2c->BusSpeed(ARM_I2C_BUS_SPEED_STANDARD)) {
      speed       = ARM_I2C_BUS_SPEED_STANDARD;
      info->speed = ARM_I2C_BUS_SPEED_STANDARD;
    }
  }
  if (speed) {
    cmd = CNTL2;
    len = i2c->SendData(SENSOR_ADDR, &cmd, sizeof(cmd), true);
    if (len < (int)sizeof(cmd)) {
      return;
    }
    len = i2c->ReceiveData(SENSOR_ADDR, &mode, sizeof(mode), false);
    if (len < (int)sizeof(mode)) {
      return;
    }
    if (mode) {
      mode <<= 1;
    } else {
      mode   = 1;
    }
    info->mode = mode;
  }
}

static uint32_t MAG_SensorModeCurrent(void)
{
  MAG_RESOURCE   *res = &MAG0_RESOURCE;
  MAG_INFO       *info;
  uint32_t        cur;

  info = res->info;
  cur  = info->mode;
  if ((UNKNOWN_MODE    == cur) ||
      (MAG_MODE_SINGLE == cur)) {
    MAG_SensorModeUpdate();
    cur = info->mode;
  }
  return cur;
}

/*! \brief       Initialize variables for sensor settings. */
static void MAG_InitializeSensorInfo(void)
{
  MAG_RESOURCE *res = &MAG0_RESOURCE;
  MAG_INFO     *info;

  info = res->info;
  info->speed   = 0;
#ifndef MAG_ENABLE_16BIT
  info->adj[0]  = 0;
  info->adj[1]  = 0;
  info->adj[2]  = 0;
#else
  info->adj[0]  = 0x80;
  info->adj[1]  = 0x80;
  info->adj[2]  = 0x80;
#endif
}

/*!
 * \brief       Get driver version.
 * \return      \ref ARM_DRV_VERSION
 */
static ARM_DRV_VERSION MAG_GetVersion(void)
{
  return DriverVersion;
}

/*!
 * \brief       Get driver capabilities.
 * \return      \ref MAG_CAPABILITIES
 */
static MAG_CAPABILITIES MAG_GetCapabilities(void)
{
  return Capabilities;
}

/*!
 * \brief       Initialize driver.
 * \return      \ref MAG_STATUS
 */
static MAG_STATUS MAG_Initialize(MAG_SignalEvent_t cb_event)
{
  MAG_RESOURCE *res = &MAG0_RESOURCE;
  MAG_INFO     *info;

  info = res->info;
  if (true == info->init) {
    return MAG_ERROR;
  }

  res->i2c->Initialize(NULL);
  info->cb    = cb_event;
  info->mode  = UNKNOWN_MODE;
  info->init  = true;
  info->power = ARM_POWER_OFF;
#if defined MAG_ENABLE_NSF | defined MAG_ENABLE_TEMPERATURE
  info->cntl1 = 0;
#endif
  MAG_InitializeSensorInfo();
  return MAG_PowerControl(ARM_POWER_LOW);
}

/*!
 * \brief       Initialize driver.
 * \return      \ref MAG_STATUS
 */
static MAG_STATUS MAG_Uninitialize(void)
{
  MAG_RESOURCE *res = &MAG0_RESOURCE;
  MAG_INFO     *info;

  MAG_PowerControl(ARM_POWER_OFF);
  res->i2c->Uninitialize();

  info = res->info;
  info->cb   = NULL;
  info->mode = UNKNOWN_MODE;
  info->init = false;
  return MAG_OK;
}

static MAG_STATUS MAG_PowerControl(ARM_POWER_STATE state)
{
  MAG_RESOURCE   *res = &MAG0_RESOURCE;
  MAG_INFO       *info;
  ARM_DRIVER_I2C *i2c;
  ARM_POWER_STATE cur;

  info = res->info;
  if (false == info->init) {
    return MAG_ERROR;
  }
  cur  = info->power;
  if (cur == state) {
    return MAG_OK;
  }

  i2c = res->i2c;
  switch (state) {
  case ARM_POWER_OFF:
    if (ARM_POWER_FULL != cur) {
      i2c->PowerControl(ARM_POWER_FULL);
    }
    MAG_ResetSensor();
    i2c->PowerControl(state);
    MAG_PowerOffSensor();
    MAG_InitializeSensorInfo();
    break;
  case ARM_POWER_LOW:
    MAG_PowerOnSensor();
    i2c->PowerControl(state);
    break;
  case ARM_POWER_FULL:
    MAG_PowerOnSensor();
    i2c->PowerControl(state);
    break;
  default:
    return MAG_ERROR;
  }
  info->power = state;
  return MAG_OK;
}

static MAG_STATUS MAG_BusSpeed(ARM_I2C_BUS_SPEED speed)
{
  MAG_RESOURCE *res = &MAG0_RESOURCE;
  MAG_INFO     *info;

  info = res->info;
  /* check precondition */
  if (ARM_POWER_FULL != info->power) {
    return MAG_ERROR;
  }
  /* check arguments */
  if ((ARM_I2C_BUS_SPEED_STANDARD != speed) &&
      (ARM_I2C_BUS_SPEED_FAST     != speed)) {
    return MAG_ERROR;
  }

  if (res->i2c->BusSpeed(speed)) {
    return MAG_ERROR_I2C_BUS;
  }
  info->speed = speed;
  return MAG_OK;
}

static MAG_STATUS MAG_SensorModeControl(MAG_MODE mode)
{
  MAG_RESOURCE *res = &MAG0_RESOURCE;
  MAG_INFO     *info;
  uint32_t      cur;

  info = res->info;
  /* check precondition */
  if (ARM_POWER_FULL != info->power) {
    return MAG_ERROR;
  }

  cur = MAG_SensorModeCurrent();
  if (cur == mode) {
    return MAG_OK;
  } else if (UNKNOWN_MODE == cur) {
    return MAG_ERROR_I2C_BUS;
  }
  switch (mode) {
  case MAG_MODE_STANDBY:
    if (MAG_OK != MAG_WriteCntl2(CNTL2_MODE_PW_DOWN)) {
      return MAG_ERROR_I2C_BUS;
    }
    /* wait until done transit 'power down mode' */
    MAG_Wait(100);
    break;
  case MAG_MODE_SINGLE:
  case MAG_MODE_CONTINUOUS_10HZ:
  case MAG_MODE_CONTINUOUS_20HZ:
  case MAG_MODE_CONTINUOUS_50HZ:
  case MAG_MODE_CONTINUOUS_100HZ:
    if (MAG_MODE_STANDBY != cur) {
      return MAG_ERROR_MODE;
    }
    if (MAG_OK != MAG_WriteCntl2(mode >> 1)) {
      return MAG_ERROR_I2C_BUS;
    }
    break;
  default:
    return MAG_ERROR;
  }
  info->mode = mode;
  return MAG_OK;
}

static MAG_STATUS MAG_LoadAdjustmentValues(void)
{
  MAG_RESOURCE   *res = &MAG0_RESOURCE;
  MAG_INFO       *info;
  uint8_t         cmd;
  ARM_DRIVER_I2C *i2c;
  uint32_t        cur;

  info = res->info;
  /* check precondition */
  if (ARM_POWER_FULL   != info->power) {
    return MAG_ERROR;
  }
  cur = MAG_SensorModeCurrent();
  if (MAG_MODE_STANDBY != cur) {
    return MAG_ERROR_MODE;
  }

  if (MAG_OK != MAG_WriteCntl2(CNTL2_MODE_ROM)) {
    return MAG_ERROR_I2C_BUS;
  }

  cmd = ASAX;
  i2c = res->i2c;
  i2c->SendData(SENSOR_ADDR, &cmd, sizeof(cmd), true);
  i2c->ReceiveData(SENSOR_ADDR, info->adj, 3, false);

  if (MAG_OK != MAG_WriteCntl2(CNTL2_MODE_PW_DOWN)) {
    return MAG_ERROR_I2C_BUS;
  }
  /* wait until done transit 'power down mode' */
  MAG_Wait(100);
  return MAG_OK;
}

static MAG_STATUS MAG_DataAvailable(void)
{
  MAG_RESOURCE   *res = &MAG0_RESOURCE;
  MAG_INFO       *info;
  uint8_t         st;

  info = res->info;
  /* check precondition */
  if (ARM_POWER_FULL != info->power) {
    return MAG_ERROR;
  }

  if (MAG_OK != MAG_ReadSt1(&st)) {
    return MAG_ERROR_I2C_BUS;
  }
  if (st & ST1_DRDY) {
    return MAG_OK;
  }
  return MAG_ERROR_NO_DATA;
}

static MAG_STATUS MAG_ReadData(int16_t *density, uint32_t *attr)
{
  int             i;
  MAG_RESOURCE   *res = &MAG0_RESOURCE;
  MAG_INFO       *info;
  ARM_DRIVER_I2C *i2c;
  uint8_t         st;
  int16_t         data[4];
  int             len;

  info = res->info;
  /* check precondition */
  if (ARM_POWER_FULL != info->power) {
    return MAG_ERROR;
  }
  /* check arguments */
  if ((NULL == density) || (NULL == attr)) {
    return MAG_ERROR;
  }

  if (MAG_OK != MAG_ReadSt1(&st)) {
    return MAG_ERROR_I2C_BUS;
  }
  if (st & ST1_DOR) {
    *attr = MAG_ATTR_OVERRUN;
  } else {
    *attr = 0;
  }
  if (0 == (st & ST1_DRDY)) {
    return MAG_ERROR_NO_DATA;
  }

  i2c = res->i2c;
  len = i2c->ReceiveData(SENSOR_ADDR, (uint8_t*)data, sizeof(data), false);
  if (len < (int)sizeof(data)) {
    return MAG_ERROR_I2C_BUS;
  }
  if (data[3] & ST2_HOFL) {
    *attr |= MAG_ATTR_OVERFLOW;
  }
  for (i = 0; i < 3; ++i) {
    #ifndef MAG_ENABLE_16BIT
    density[i] = ((int32_t)data[i] * ((int32_t)info->adj[i] + 128)) / 128;
    #else
    density[i] = ((int32_t)data[i] * ((int32_t)info->adj[i] + 128)) / 256;
    #endif
  }
  
#ifdef MAG_ENABLE_TEMPERATURE
  if( info->cntl1 & CNTL1_BIT_TEM ){
    density[3] = data[3]&0xff;
  }
#endif
  return MAG_OK;
}

static MAG_STATUS MAG_RunSelfTest(void)
{
  int             i;
  MAG_RESOURCE   *res = &MAG0_RESOURCE;
  MAG_INFO       *info;
  ARM_DRIVER_I2C *i2c;
  int16_t         data[4];
  int16_t         val;
  int             len;
  uint8_t         st;
  MAG_STATUS      err;

  err = MAG_LoadAdjustmentValues();
  if (MAG_OK != err) {
    return err;
  }

  if (MAG_OK != MAG_WriteCntl2(CNTL2_MODE_SELFTEST)) {
    return MAG_ERROR_I2C_BUS;
  }

  do {
    if (MAG_OK != MAG_ReadSt1(&st)) {
      return MAG_ERROR_I2C_BUS;
    }
  } while (0 == (st & ST1_DRDY));

  i2c = res->i2c;
  len = i2c->ReceiveData(SENSOR_ADDR, (uint8_t*)data, sizeof(data), false);
  if (len < (int)sizeof(data)) {
    return MAG_ERROR_I2C_BUS;
  }

  if (MAG_OK != MAG_WriteCntl2(CNTL2_MODE_PW_DOWN)) {
    return MAG_ERROR_I2C_BUS;
  }

  info = res->info;
  for (i = 0; i < 3; ++i) {
    data[i] = ((int32_t)data[i] * 128 + info->adj[i] + 128) / 128;
  }

  /* wait until done transit 'power down mode' */
  MAG_Wait(100);

  /* check results */
#ifndef MAG_ENABLE_16BIT
  for (i = 0; i < 2; ++i) {
    val = data[i];
    if ((val < -30) || (30 < val)) {
      return MAG_ERROR_TEST;
    }
  }
  val = data[2];
  if ((val < -400) || (-50 < val)) {
    return MAG_ERROR_TEST;
  }
#else /* MAG_ENABLE_16BIT */
  val = data[0];
  if((val < -200)  || (val > 200) ){
    return MAG_ERROR_TEST;
  }
  val = data[1];
  if((val < -200)  || (val > 200) ){
    return MAG_ERROR_TEST;
  }
  val = data[2];
  if((val < -1600) || (val > -400) ){
    return MAG_ERROR_TEST;
  }
#endif /* MAG_ENABLE_16BIT */
  
  return MAG_OK;
}

#ifdef MAG_ENABLE_NSF
static MAG_STATUS MAG_SetNoiseSuppressionFilter(MAG_NSF nsf)
{
  MAG_RESOURCE   *res = &MAG0_RESOURCE;

  if( nsf > MAG_NSF_HIGH ) {
    return MAG_ERROR;
  }
  
  if (ARM_POWER_FULL   != res->info->power) {
    return MAG_ERROR;
  }
  if (MAG_MODE_STANDBY != MAG_SensorModeCurrent()) {
    return MAG_ERROR_MODE;
  }

  res->info->cntl1 = (res->info->cntl1 & ~(CNTL1_BIT_NSF)) | (nsf << 5);
  return MAG_WriteCntl1(res->info->cntl1);
}
#endif

#ifdef MAG_ENABLE_TEMPERATURE
static MAG_STATUS MAG_SetTempMeasurement(bool enable)
{
  MAG_RESOURCE *res = &MAG0_RESOURCE;

  if (ARM_POWER_FULL   != res->info->power) {
    return MAG_ERROR;
  }
  if (MAG_MODE_STANDBY != MAG_SensorModeCurrent()) {
    return MAG_ERROR_MODE;
  }

  res->info->cntl1 = (res->info->cntl1&~(CNTL1_BIT_TEM)) | ((!!enable)<<7);
  return MAG_WriteCntl1( res->info->cntl1);
}
#endif

#ifdef MAG_ENABLE_INTERRUPT
static void MAG_InterruptHandler(uint32_t pin)
{
  MAG_RESOURCE *res = &MAG0_RESOURCE;
  
  if( NULL != res->info->cb ){
    res->info->cb(MAG_EVENT_DATA_READY);
  }

  return;
}

static MAG_STATUS MAG_EnableInterrupt(void)
{
  MAG_RESOURCE *res = &MAG0_RESOURCE;
  TZ10XX_DRIVER_GPIO *gpio = res->gpio;
  GPIO_STATUS err;
  
  err = gpio->Configure(MAG_GPIO_INT_PIN, GPIO_DIRECTION_INPUT_HI_Z,
                        GPIO_EVENT_EDGE_POS, MAG_InterruptHandler);
  if (err) {
    return MAG_ERROR_GPIO;
  }

  return MAG_OK;
}

static MAG_STATUS MAG_DisableInterrupt(void)
{
  MAG_RESOURCE *res = &MAG0_RESOURCE;
  TZ10XX_DRIVER_GPIO *gpio = res->gpio;
  GPIO_STATUS err;
  
  err = gpio->Configure(MAG_GPIO_INT_PIN, GPIO_DIRECTION_INPUT_HI_Z,
                        GPIO_EVENT_DISABLE, NULL);
  if (err) {
    return MAG_ERROR_GPIO;
  }

  return MAG_OK;
}

static MAG_STATUS MAG_SetDataReadyInterrupt(bool enable)
{
  MAG_RESOURCE *res = &MAG0_RESOURCE;

  if (ARM_POWER_FULL   != res->info->power) {
    return MAG_ERROR;
  }
  if (MAG_MODE_STANDBY != MAG_SensorModeCurrent()) {
    return MAG_ERROR_MODE;
  }

  if( true == enable ){
    return MAG_EnableInterrupt();
  } else{
    return MAG_DisableInterrupt();
  }
}
#endif

TZ10XX_DRIVER_MAG Driver_MAG = {
  MAG_GetVersion,
  MAG_GetCapabilities,
  MAG_Initialize,
  MAG_Uninitialize,
  MAG_PowerControl,
  MAG_BusSpeed,
  MAG_SensorModeControl,
  MAG_LoadAdjustmentValues,
  MAG_DataAvailable,
  MAG_ReadData,
  MAG_RunSelfTest,
#ifdef MAG_ENABLE_NSF
  MAG_SetNoiseSuppressionFilter,
#endif
#ifdef MAG_ENABLE_TEMPERATURE
  MAG_SetTempMeasurement,
#endif
#ifdef MAG_ENABLE_INTERRUPT
  MAG_SetDataReadyInterrupt,
#endif
};
#endif
