/**
 * @file RNG_TZ10xx.c
 * @brief a header file for TZ10xx RNG driver
 * @date $Date:: 2014-11-19 15:03:19 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#include "RNG_TZ10xx.h"
#include "RTE_Device.h"
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"

#define RNG_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 15)   /* driver version */

static RNG_STATUS RNG_Read (uint32_t *rn);

/* RNG Initialize status */
typedef enum _RNG_INIT_STATUS {
  RNG_UN_INITIALIZED    = 0,     
  RNG_INITIALIZED= 1,  
} RNG_INIT_STATUS;

 /* RNG Information  */
typedef struct {
	RNG_INIT_STATUS           init;    // Initialize State
	ARM_POWER_STATE         power;                // Power mode
} RNG_INFO;

/* RNG Resources definition */
typedef struct {
  RNG_INFO         *info;                 //< RNG information
} const RNG_RESOURCES;

static RNG_INFO rng_info = {
	RNG_UN_INITIALIZED,
	ARM_POWER_OFF,
};

static RNG_RESOURCES rng_res = {	
	&rng_info
};

/**
  \fn          static void rng_resetOff(void)
  \brief       Reset off RNG.
\return      void
*/
static void rng_resetOff(void)
{
	Driver_PMU.EnableModule(PMU_MODULE_RNG, 1);
	return;
}

/**
  \fn          static void rng_resetOn(void)
  \brief       Reset on RNG.
\return      void
*/
static void rng_resetOn(void)
{
	Driver_PMU.EnableModule(PMU_MODULE_RNG, 0);
	return;
}

/**
  \fn          static RNG_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION RNG_GetVersion(void)
{
  static const ARM_DRIVER_VERSION driver_version = {
    RNG_API_VERSION,
    RNG_DRV_VERSION
  };
  return driver_version;
}

/**
  \fn          static RNG_CAPABILITIES RNG_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref RNG_CAPABILITIES
*/
static RNG_CAPABILITIES RNG_GetCapabilities(void)
{
  RNG_CAPABILITIES capabilities = {0};
	return capabilities;
}

/**
  \fn          static RNG_STATUS RNG_PowerControl (ARM_POWER_STATE  state)
  \brief       Controls RNG Interface Power.
  \param[in]   state    Power state
  \return      \ref RNG_STATUS
*/
static RNG_STATUS RNG_PowerControl (ARM_POWER_STATE  state)
{
	ARM_POWER_STATE  pre_state = rng_res.info->power;
	
 if(RNG_UN_INITIALIZED == rng_res.info->init)	return RNG_ERROR;
	
 if((ARM_POWER_FULL == pre_state) && (ARM_POWER_FULL != state)) {
	volatile uint32_t tmp;
	while(1) {
		tmp = rng->RNDREADY_b.RNDREADY;
		if(tmp) break;
	}
}
	 
 switch (state) {
    case ARM_POWER_OFF:
     if(ARM_POWER_OFF != pre_state) {
					rng_resetOn();
					rng_res.info->power  = ARM_POWER_OFF;
			}
      break;
    case ARM_POWER_LOW:
    case ARM_POWER_FULL:
       if(ARM_POWER_OFF == pre_state) {
					rng_resetOff();
			}
			rng_res.info->power = state;
			break;
 		default:
			return RNG_ERROR;
	}
 
  if((ARM_POWER_FULL != pre_state) && (ARM_POWER_FULL == state)) {
		uint32_t i;
		uint32_t rn;
		/* Dummy Read x 6 words */
		for(i=0;i<6;i++) {
			RNG_Read(&rn);
		}
	}
	
  return RNG_OK;
}

/**
  \fn          static RNG_STATUS RNG_Initialize (void)
  \brief       Initialize RNG Interface.
  \return      \ref RNG_STATUS
*/
static RNG_STATUS RNG_Initialize (void)
{
	if (RNG_UN_INITIALIZED != rng_res.info->init) return RNG_ERROR;

	rng_res.info->init = RNG_INITIALIZED;
	rng_resetOn();
	
	return RNG_PowerControl(ARM_POWER_LOW);
}

/**
  \fn          static RNG_STATUS RNG_Uninitialize (void)
  \brief       De-initialize RNG Interface.
  \return      \ref RNG_STATUS
*/
static RNG_STATUS RNG_Uninitialize (void)
{
	if(RNG_UN_INITIALIZED != rng_res.info->init) {
		RNG_PowerControl(ARM_POWER_OFF);
	}

	rng_res.info->init = RNG_UN_INITIALIZED;

  return RNG_OK;
}

/**
  \fn          static RNG_STATUS RNG_Read (uint32_t *rn)
  \brief       read random number.
	\param[out]   rn    pointer to random number
 \return      \ref RNG_STATUS
*/
static RNG_STATUS RNG_Read (uint32_t *rn)
{
	volatile uint32_t tmp;
	
	if(RNG_UN_INITIALIZED == rng_res.info->init)	return RNG_ERROR;
	if(ARM_POWER_FULL != rng_res.info->power) return RNG_ERROR;
	
	while(1) {
		tmp = rng->RNDREADY_b.RNDREADY;
		if(tmp) break;
	}
	
	*rn = rng->RNDO;
	return RNG_OK;
}

/* RNG Driver Control Block */
TZ10XX_DRIVER_RNG Driver_RNG = {
	RNG_GetVersion,
	RNG_GetCapabilities,
	RNG_Initialize,
	RNG_Uninitialize,
	RNG_PowerControl ,
	RNG_Read
};
 
