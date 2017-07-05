/**
 * @file GPIO_TZ10xx.c
 * @brief a header file for TZ10xx GPIO driver
 * @date $Date:: 2016-02-17 15:17:41 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#include "GPIO_TZ10xx.h"
#include "RTE_Device.h"
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"

#define GPIO_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 41)   /* driver version */

/* Configuration settings for Driver_GPIO */
static const uint32_t   GPIO_CONNECT_TABLE  =
	( ((RTE_GPIO_0_ID!=0)<<0) |     
	 ((RTE_GPIO_1_ID!=0)<<1) |     
	 ((RTE_GPIO_2_ID!=0)<<2) |      
	 ((RTE_GPIO_3_ID!=0)<<3) |      
	 ((RTE_GPIO_4_ID!=0)<<4) |      
	 ((RTE_GPIO_5_ID!=0)<<5) |      
	 ((RTE_GPIO_6_ID!=0)<<6) |      
	 ((RTE_GPIO_7_ID!=0)<<7) |      
	 ((RTE_GPIO_8_ID!=0)<<8) |     
	 ((RTE_GPIO_9_ID!=0)<<9) |      
	 ((RTE_GPIO_10_ID!=0)<<10) |      
	 ((RTE_GPIO_11_ID!=0)<<11) |     
	 ((RTE_GPIO_12_ID!=0)<<12) |     
	 ((RTE_GPIO_13_ID!=0)<<13) |      
	 ((RTE_GPIO_14_ID!=0)<<14) |      
	 ((RTE_GPIO_15_ID!=0)<<15) |      
	 ((RTE_GPIO_16_ID!=0)<<16) |      
	 ((RTE_GPIO_17_ID!=0)<<17) |      
	 ((RTE_GPIO_18_ID!=0)<<18) |      
	 ((RTE_GPIO_19_ID!=0)<<19) |      
	 ((RTE_GPIO_20_ID!=0)<<20) |      
	 ((RTE_GPIO_21_ID!=0)<<21) |      
	 ((RTE_GPIO_22_ID!=0)<<22) |      
	 ((RTE_GPIO_23_ID!=0)<<23) |      
	 ((RTE_GPIO_24_ID!=0)<<24) |      
	 ((RTE_GPIO_25_ID!=0)<<25) |      
	 ((RTE_GPIO_26_ID!=0)<<26) |      
	 ((RTE_GPIO_27_ID!=0)<<27) |      
	 ((RTE_GPIO_28_ID!=0)<<28) |     
	 ((RTE_GPIO_29_ID!=0)<<29) |     
	 ((RTE_GPIO_30_ID!=0)<<30) |      
	 ((uint32_t)(RTE_GPIO_31_ID!=0)<<31) )  ;
 
 #define GPIO_PIN_ID(pin)  ((GPIO_CONNECT_TABLE>>pin)&0x1)
 
/* GPIO Initialize status */
typedef enum _GPIO_INIT_STATUS {
  GPIO_UN_INITIALIZED    = 0,     
  GPIO_INITIALIZED= 1,  
} GPIO_INIT_STATUS;

 /* GPIO Information  */
typedef struct {
	GPIO_INIT_STATUS           init;    // Initialize State
	ARM_POWER_STATE         power;                // Power mode
  GPIO_SignalEvent_t         	 cb_event[32];         // Event Callback
	uint32_t                 				   nvic_flg;           //NVIC enable flag
} GPIO_INFO;

/* GPIO Resources definition */
typedef struct {
  GPIO_INFO         *info;                 //< GPIO information
} const GPIO_RESOURCES;

static GPIO_INFO gpio_info = {
	GPIO_UN_INITIALIZED,
	ARM_POWER_OFF,
};

static GPIO_RESOURCES gpio_res = {	
	&gpio_info
};

/**
  \fn          static IRQn_Type GPIO_GetIRQn (uint32_t pin)
  \brief       Get IRQn number.
  \param[in]   pin      pin number
 \return      \ref IRQn_Type
*/
static IRQn_Type GPIO_GetIRQn (uint32_t pin)
{
#if 0
	switch(pin) {
		case 0: 	return GPIO_Pin0_IRQn;
		case 1: 	return GPIO_Pin1_IRQn;
		case 2: 	return GPIO_Pin2_IRQn;
		case 3: 	return GPIO_Pin3_IRQn;
		case 4: 	return GPIO_Pin4_IRQn;
		case 5: 	return GPIO_Pin5_IRQn;
		case 6: 	return GPIO_Pin6_IRQn;
		case 7: 	return GPIO_Pin7_IRQn;
		case 8: 	return GPIO_Pin8_IRQn;
		case 9: 	return GPIO_Pin9_IRQn;
		case 10: 	return GPIO_Pin10_IRQn;
		case 11: 	return GPIO_Pin11_IRQn;
		case 12: 	return GPIO_Pin12_IRQn;
		case 13: 	return GPIO_Pin13_IRQn;
		case 14: 	return GPIO_Pin14_IRQn;
		case 15: 	return GPIO_Pin15_IRQn;
		case 16: 	return GPIO_Pin16_IRQn;
		case 17: 	return GPIO_Pin17_IRQn;
		case 18: 	return GPIO_Pin18_IRQn;
		case 19: 	return GPIO_Pin19_IRQn;
		case 20: 	return GPIO_Pin20_IRQn;
		case 21: 	return GPIO_Pin21_IRQn;
		case 22: 	return GPIO_Pin22_IRQn;
		case 23: 	return GPIO_Pin23_IRQn;
		case 24: 	return GPIO_Pin24_IRQn;
		case 25: 	return GPIO_Pin25_IRQn;
		case 26: 	return GPIO_Pin26_IRQn;
		case 27: 	return GPIO_Pin27_IRQn;
		case 28: 	return GPIO_Pin28_IRQn;
		case 29: 	return GPIO_Pin29_IRQn;
		case 30: 	return GPIO_Pin30_IRQn;
		case 31: 
		default:  return GPIO_Pin31_IRQn;
	}
#else
  return (IRQn_Type) pin;
#endif
}


/**
  \fn          static void GPIO_ResetOff(void)
  \brief       Reset off GPIO.
\return      void
*/
static void GPIO_ResetOff(void)
{
	Driver_PMU.EnableModule(PMU_MODULE_GPIO0, 1);
	Driver_PMU.EnableModule(PMU_MODULE_GPIO1, 1);
	Driver_PMU.EnableModule(PMU_MODULE_GPIO2, 1);
	Driver_PMU.EnableModule(PMU_MODULE_GPIO3, 1);
	return;
}

/**
  \fn          static void GPIO_ResetOn(void)
  \brief       Reset on GPIO.
\return      void
*/
static void GPIO_ResetOn(void)
{
	Driver_PMU.EnableModule(PMU_MODULE_GPIO0, 0);
	Driver_PMU.EnableModule(PMU_MODULE_GPIO1, 0);
	Driver_PMU.EnableModule(PMU_MODULE_GPIO2, 0);
	Driver_PMU.EnableModule(PMU_MODULE_GPIO3, 0);
	return;
}

/**
  \fn          static void GPIO_SetIO(uint32_t pin, GPIO_DIRECTION dir)
  \brief       Set IO register in gconf.
  \param[in]   pin      pin number
  \param[in]   dir      pin direction
  \return      void
*/
static void GPIO_SetIO(uint32_t pin, GPIO_DIRECTION dir)
{
	PMU_IO_FUNC func = (PMU_IO_FUNC) pin;
	PMU_IO_DRIVE_CAPABILITY dc = PMU_DRIVE_CAPABILITY_2MA;
	PMU_IO_RESISTOR res = PMU_IO_RESISTOR_NONE;
	
	if(0 == pin) return; 						/* IO register can not be changed at pin0. */

	switch(dir) {
		case GPIO_DIRECTION_INPUT_HI_Z:
			break;
		case GPIO_DIRECTION_INPUT_PULL_UP:
			res = PMU_IO_RESISTOR_PULLUP; 
			break;
	  case GPIO_DIRECTION_INPUT_PULL_DOWN:
			res = PMU_IO_RESISTOR_PULLDOWN;
			break;
/*		case GPIO_DIRECTION_OUTPUT_2MA:
			break;*/
		case GPIO_DIRECTION_OUTPUT_4MA:
			dc = PMU_DRIVE_CAPABILITY_4MA; 
			break;
		case GPIO_DIRECTION_OUTPUT_5MA:
			dc = PMU_DRIVE_CAPABILITY_5MA;
			break;
		case GPIO_DIRECTION_OUTPUT_7MA:
			dc = PMU_DRIVE_CAPABILITY_7MA;
			break;
		default:
			break;
	}
	Driver_PMU.ConfigureIOCell(func, dc, res);
}

/**
  \fn          static void GPIO_SetNVICEnable(uint32_t pin)
  \brief       set NVIC of pin enable.
  \param[in]   pin      pin number
\return      void
*/
static void GPIO_SetNVICEnable(uint32_t pin)
{
	pin &= 0x1f;
	if(0 == (gpio_res.info->nvic_flg&(1<<pin))) {
			NVIC_ClearPendingIRQ(GPIO_GetIRQn (pin));
			NVIC_EnableIRQ(GPIO_GetIRQn (pin));
			gpio_res.info->nvic_flg |= 1<<pin;
	}
	return;
}
/**
  \fn          static void GPIO_SetNVICDisable(uint32_t pin)
  \brief       set NVIC of pin disable.
  \param[in]   pin      pin number
\return      void
*/
static void GPIO_SetNVICDisable(uint32_t pin)
{
	pin &= 0x1f;
	if( (gpio_res.info->nvic_flg&(1<<pin))) {
				NVIC_DisableIRQ(GPIO_GetIRQn (pin));
				NVIC_ClearPendingIRQ(GPIO_GetIRQn (pin));
				gpio_res.info->nvic_flg &= ~(1<<pin);
	}
	return;
}

/**
  \fn          static gpio_Type GPIO_GetPointer ( uint32_t pin, uint32_t *gpiopin)
  \brief       Get GPIO peripheral pointer & pin number.
  \param[in]   pin      pin number
  \param[out]   gpiopin      N'th GPIO pin
  \return       \ref gpio_Type 
*/
static gpio_Type *GPIO_GetPointer ( uint32_t pin, uint32_t *gpiopin)
{
	*gpiopin = pin&0x7;
	
 	switch(pin>>3) {
		case 0:
			return gpio0;
		case 1:
			return gpio1;
		case 2:
			return gpio2;
		case 3:
		default:
			return gpio3;
	}
}

/**
  \fn          static void GPIO_SetBit ( uint32_t pin, uint8_t val, volatile uint32_t *regptr)
  \brief       Set register bit value.
  \param[in]   pin      pin number
  \param[in]   val      writing value
  \param[in]   regptr     register address
  \return      void 
*/
static void GPIO_SetBit ( uint32_t pin, uint8_t val, volatile uint32_t *regptr)
{
	pin &= 0x1f;
	val &= 0x1;
	BITBAND_VALUE(regptr, pin) = val;
	return;
}

/**
  \fn          static void GPIO_IRQHandler (uint32_t pin)
  \brief       Interrupt handler.
  \param[in]   pin    pin number
  \return      void 
*/
static void GPIO_IRQHandler(uint32_t pin)
{
	uint32_t gpiopin;
	gpio_Type *gpiox;

	pin &= 0x1f;
 	gpiox = GPIO_GetPointer(pin, &gpiopin);
	GPIO_SetBit ( gpiopin, 1, &gpiox->GPIOIC); /* clear interrupt */
  if (NULL != gpio_res.info->cb_event[pin]) {
 	    gpio_res.info->cb_event[pin](pin);
   }
 }
	
	
/**
  \fn          static GPIO_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION GPIO_GetVersion(void)
{
  static const ARM_DRIVER_VERSION driver_version = {
    GPIO_API_VERSION,
    GPIO_DRV_VERSION
  };
  return driver_version;
}

/**
  \fn          static GPIO_CAPABILITIES GPIO_GetCapabilities (uint32_t pin)
  \brief       Get driver capabilities.
  \param[in]   pin      pin number
  \return      \ref GPIO_CAPABILITIES
*/
static GPIO_CAPABILITIES GPIO_GetCapabilities(uint32_t pin)
{
  GPIO_CAPABILITIES capabilities = {0};
	/* set capabilities */
#if 0
	switch(pin) {
		case 0:
			capabilities.connected = GPIO_PIN_ID(pin);
			capabilities.resistor = 0; 
			capabilities.output = 0; 
			capabilities.wake_up = 1; 
			capabilities.event = 1; 
		break;

		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 24:
		case 25:
		case 26:
		case 27:
		case 30:
			capabilities.connected =  GPIO_PIN_ID(pin);
			capabilities.resistor = 1; 
			capabilities.output = 1; 
			capabilities.wake_up = 1; 
			capabilities.event = 1; 
		break;
		
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
		case 16:
		case 17:
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
		case 28:
		case 29:
		case 31:
			capabilities.connected =  GPIO_PIN_ID(pin);
			capabilities.resistor = 1; 
			capabilities.output = 1; 
			capabilities.wake_up = 0; 
			capabilities.event = 1; 
		break;

   default:
			capabilities.connected = 0; 
			capabilities.resistor = 0; 
			capabilities.output = 0; 
			capabilities.wake_up = 0; 
			capabilities.event = 0; 
		break;
 	}
#else
	capabilities.resistor = 0; 
	capabilities.output = 0; 
	capabilities.wake_up = 0; 
  if(pin > 31) {
    capabilities.connected = 0; 
    capabilities.event = 0;
    return capabilities;   
  }    
	capabilities.connected = GPIO_PIN_ID(pin);
	capabilities.event = 1; 
  if(pin == 0) {
			capabilities.wake_up = 1; 
  } else {
			capabilities.resistor = 1; 
			capabilities.output = 1; 
      if((pin < 8) || ((pin > 23) && (pin < 28)) || (pin == 30)) {
        capabilities.wake_up = 1; 
      }
  }
#endif
	return capabilities;
}

/**
  \fn          static GPIO_STATUS GPIO_PowerControl (ARM_POWER_STATE  state)
  \brief       Controls GPIO Interface Power.
  \param[in]   state    Power state
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_PowerControl (ARM_POWER_STATE  state)
{
 if(GPIO_UN_INITIALIZED == gpio_res.info->init)	return GPIO_ERROR;
	
 switch (state) {
    case ARM_POWER_OFF:
     if(ARM_POWER_OFF != gpio_res.info->power) {
					GPIO_ResetOn();
					gpio_res.info->power  = ARM_POWER_OFF;
			}
      break;
    case ARM_POWER_LOW:
    case ARM_POWER_FULL:
       if(ARM_POWER_OFF == gpio_res.info->power) {
					GPIO_ResetOff();
			}
			gpio_res.info->power = state;
			break;
 		default:
			return GPIO_ERROR;
	}
  return GPIO_OK;
}

/**
  \fn          static GPIO_STATUS GPIO_Initialize (void)
  \brief       Initialize GPIO Interface.
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Initialize (void)
{
	 uint32_t pin;

	if (GPIO_UN_INITIALIZED != gpio_res.info->init) return GPIO_ERROR;

	gpio_res.info->init = GPIO_INITIALIZED;
	GPIO_ResetOn();
	
	// Clear param
	for(pin=0;pin<32;pin++) {
		gpio_res.info->cb_event[pin] = NULL;
	}
	gpio_res.info->nvic_flg = 0;
	
	return GPIO_PowerControl(ARM_POWER_LOW);
}

/**
  \fn          static GPIO_STATUS GPIO_Uninitialize (void)
  \brief       De-initialize GPIO Interface.
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Uninitialize (void)
{
  uint32_t pin;
	
	if (GPIO_UN_INITIALIZED != gpio_res.info->init) {
		GPIO_PowerControl(ARM_POWER_OFF);
	}

  // Clear param
 	for(pin=0;pin<32;pin++) {
		GPIO_SetNVICDisable(pin); 
		gpio_res.info->cb_event[pin] = NULL;
	}
	gpio_res.info->init = GPIO_UN_INITIALIZED;

  return GPIO_OK;
}

/**
  \fn          static GPIO_STATUS GPIO_Configure(uint32_t pin, GPIO_DIRECTION dir, GPIO_EVENT event, GPIO_SignalEvent_t cb_event)
  \brief       Configure GPIO pin.
  \param[in]   pin      pin number
  \param[in]   dir      GPIO direction
  \param[in]   event      GPIO event
  \param[in]   cb_event     Pointer to GPIO_SignalEvent
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Configure(uint32_t pin, GPIO_DIRECTION dir, GPIO_EVENT event,
																											GPIO_SignalEvent_t cb_event)
{
  GPIO_CAPABILITIES capabilities;
	uint8_t   gpiodir = 0; /* input */
	uint8_t   gpiois = 0; /* edge */
	uint8_t   gpioibe = 0; /* either */
	uint8_t   gpioiev = 0; /* low or neg */
	uint8_t   gpioie = 1; /* interrupt enable */
	uint32_t gpiopin;
	gpio_Type *gpiox;
		
	if(GPIO_UN_INITIALIZED == gpio_res.info->init)	return GPIO_ERROR;
	if (ARM_POWER_FULL != gpio_res.info->power) return GPIO_ERROR;
	if (31 < pin) return GPIO_ERROR;
	if(0 ==  GPIO_PIN_ID(pin)) return GPIO_ERROR;

 	capabilities = GPIO_GetCapabilities(pin);
	switch(dir) {
		case GPIO_DIRECTION_INPUT_HI_Z: /* gpiodir=0 */
			break;
		case GPIO_DIRECTION_INPUT_PULL_UP: /* gpiodir=0 */
		case GPIO_DIRECTION_INPUT_PULL_DOWN: /* gpiodir=0 */
			if(0 ==  capabilities.resistor) return GPIO_ERROR;
			break;
		case GPIO_DIRECTION_OUTPUT_2MA: /* gpiodir=1 */
		case GPIO_DIRECTION_OUTPUT_4MA: /* gpiodir=1 */
		case GPIO_DIRECTION_OUTPUT_5MA: /* gpiodir=1 */
		case GPIO_DIRECTION_OUTPUT_7MA: /* gpiodir=1 */
			if(0 ==  capabilities.output) return GPIO_ERROR;
			gpiodir = 1; /* output */
			break;
		default:
			return GPIO_ERROR;
	 }
	
	 switch(event) {
		case GPIO_EVENT_DISABLE:      /* gpiois=u  gpioibe=u  gpioiev=u  gpioiie=0 */
			gpioie = 0; /* interrupt disable */	
			break;
		case GPIO_EVENT_LEVEL_HIGH: /* gpiois=1  gpioibe=0  gpioiev=1  gpioiie=1 */
			gpioiev = 1; /* high or pos */
			/* fall through */
		case GPIO_EVENT_LEVEL_LOW: /* gpiois=1  gpioibe=0  gpioiev=0  gpioiie=1 */
			gpiois = 1; /* level */
			break;
		case GPIO_EVENT_EDGE_POS:   /* gpiois=0  gpioibe=0  gpioiev=1  gpioiie=1 */
			gpioiev = 1; /* high or pos */
			/* fall through */
		case GPIO_EVENT_EDGE_NEG:   /* gpiois=0  gpioibe=0  gpioiev=0  gpioiie=1 */
			break;
		case GPIO_EVENT_EDGE_BOTH: /* gpiois=0  gpioibe=1  gpioiev=0  gpioiie=1 */
			gpioibe = 1; /* both */
			break;
		default:
			return GPIO_ERROR;
	 }
	
		/* configure setting */
 	gpiox = GPIO_GetPointer(pin, &gpiopin);
	GPIO_SetBit ( gpiopin, 0, &gpiox->GPIOIE); /* set disable before setting registers */
	GPIO_SetIO( pin, dir);
	GPIO_SetBit ( gpiopin, gpiodir, &gpiox->GPIODIR);
	if(0 == gpiodir && gpioie) { /* enable  and input */
	    gpio_res.info->cb_event[pin] = cb_event;
			GPIO_SetNVICEnable(pin); 
			GPIO_SetBit ( gpiopin, gpiois, &gpiox->GPIOIS);
			GPIO_SetBit ( gpiopin, gpioibe, &gpiox->GPIOIBE);
			GPIO_SetBit ( gpiopin, gpioiev, &gpiox->GPIOIEV);
			GPIO_SetBit ( gpiopin, 1, &gpiox->GPIOIC); /* clear interrupt */
			GPIO_SetBit ( gpiopin, gpioie, &gpiox->GPIOIE);
	}
	else { /* disable or output */
			GPIO_SetBit ( gpiopin, 1, &gpiox->GPIOIC); /* clear interrupt */
			GPIO_SetNVICDisable(pin); 
	}			
	
	return GPIO_OK;
}
 
/**
  \fn          static GPIO_STATUS GPIO_ReadPin (uint32_t pin, uint32_t *val)
  \brief       Read value of a gpiopin.
  \param[in]   pin      pin number
  \param[out]  val      read value
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_ReadPin(uint32_t pin, uint32_t *val)
{
	volatile uint32_t   ret;
	uint32_t gpiopin;
	gpio_Type *gpiox;
	
	if(GPIO_UN_INITIALIZED == gpio_res.info->init)	return GPIO_ERROR;
	if (ARM_POWER_FULL != gpio_res.info->power) return GPIO_ERROR;
	if (31 < pin) return GPIO_ERROR;

	if(0 ==  GPIO_PIN_ID(pin)) {
		*val = 0;
	}
	else  {
		gpiox = GPIO_GetPointer(pin, &gpiopin);
		ret = *(gpiox->GPIODATA + (1<<gpiopin));
		*val = (ret>>gpiopin)&0x1;
	}
	return GPIO_OK;
}
 
/**
  \fn          static GPIO_STATUS GPIO_Read ( uint32_t *val)
  \brief       Read  value of all gpiopins.
  \param[out]  val      read value
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Read(uint32_t *val)
{
	volatile uint32_t   ret;
	uint32_t data = 0;
	
	if(GPIO_UN_INITIALIZED == gpio_res.info->init)	return GPIO_ERROR;
	if (ARM_POWER_FULL != gpio_res.info->power) return GPIO_ERROR;
	
	ret = *(gpio0->GPIODATA + ((GPIO_CONNECT_TABLE&0x000000ff)>>0));
	data |= (ret&0x000000ff);
	ret = *(gpio1->GPIODATA + ((GPIO_CONNECT_TABLE&0x0000ff00)>>8));
	data |= (ret&0x000000ff)<<8;
	ret = *(gpio2->GPIODATA + ((GPIO_CONNECT_TABLE&0x00ff0000)>>16));
	data |= (ret&0x000000ff)<<16;
	ret = *(gpio3->GPIODATA + ((GPIO_CONNECT_TABLE&0xff000000)>>24));
	data |= (ret&0x000000ff)<<24;
	
	*val = data;
	return GPIO_OK;
}
 
/**
  \fn          static GPIO_STATUS GPIO_WritePin ( uint32_t pin, uint32_t val)
  \brief       write value of a gpiopin.
  \param[in]  pin    pin number
  \param[in]  val      writing value
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_WritePin( uint32_t pin, uint32_t val)
{
	uint32_t gpiopin;
	gpio_Type *gpiox;
	
	if(GPIO_UN_INITIALIZED == gpio_res.info->init)	return GPIO_ERROR;
	if (ARM_POWER_FULL != gpio_res.info->power) return GPIO_ERROR;
	if (31 < pin) return GPIO_ERROR;
	if (1 < val) return GPIO_ERROR;

	if(0 ==  GPIO_PIN_ID(pin)) {
		/* do nothing */
	}
	else  {
		gpiox = GPIO_GetPointer(pin, &gpiopin);
		*(gpiox->GPIODATA + (1<<gpiopin)) = (val&0x1)<<gpiopin;
	}
	
	return GPIO_OK;
}
 
/**
  \fn          static GPIO_STATUS GPIO_Write ( uint32_t mask, uint32_t val)
  \brief       write value of all gpiopins.
  \param[in]  mask    pin mask
  \param[in]  val      writing value
  \return      \ref GPIO_STATUS
*/
static GPIO_STATUS GPIO_Write( uint32_t mask, uint32_t val)
{
	uint32_t  pinmask = GPIO_CONNECT_TABLE & mask;
	
	if(GPIO_UN_INITIALIZED == gpio_res.info->init)	return GPIO_ERROR;
	if (ARM_POWER_FULL != gpio_res.info->power) return GPIO_ERROR;
	
	*(gpio0->GPIODATA + ((pinmask&0x000000ff)>>0)) = (val>>0)&0x000000ff;
	*(gpio1->GPIODATA + ((pinmask&0x0000ff00)>>8)) = (val>>8)&0x000000ff;
	*(gpio2->GPIODATA + ((pinmask&0x00ff0000)>>16)) = (val>>16)&0x000000ff;
	*(gpio3->GPIODATA + ((pinmask&0xff000000)>>24)) = (val>>24)&0x000000ff;
	
	return GPIO_OK;
}
 
/* interrupt functions */

#if (RTE_GPIO_0_ID != 0)
void GPIO_Pin0_IRQHandler(void)
{
  GPIO_IRQHandler(0);
}
#endif

#if (RTE_GPIO_1_ID != 0)
void GPIO_Pin1_IRQHandler(void)
{
  GPIO_IRQHandler(1);
}
#endif

#if (RTE_GPIO_2_ID != 0)
void GPIO_Pin2_IRQHandler(void)
{
  GPIO_IRQHandler(2);
}
#endif

#if (RTE_GPIO_3_ID != 0)
void GPIO_Pin3_IRQHandler(void)
{
  GPIO_IRQHandler(3);
}
#endif

#if (RTE_GPIO_4_ID != 0)
void GPIO_Pin4_IRQHandler(void)
{
  GPIO_IRQHandler(4);
}
#endif

#if (RTE_GPIO_5_ID != 0)
void GPIO_Pin5_IRQHandler(void)
{
  GPIO_IRQHandler(5);
}
#endif

#if (RTE_GPIO_6_ID != 0)
void GPIO_Pin6_IRQHandler(void)
{
  GPIO_IRQHandler(6);
}
#endif

#if (RTE_GPIO_7_ID != 0)
void GPIO_Pin7_IRQHandler(void)
{
  GPIO_IRQHandler(7);
}
#endif

#if (RTE_GPIO_8_ID != 0)
void GPIO_Pin8_IRQHandler(void)
{
  GPIO_IRQHandler(8);
}
#endif

#if (RTE_GPIO_9_ID != 0)
void GPIO_Pin9_IRQHandler(void)
{
  GPIO_IRQHandler(9);
}
#endif

#if (RTE_GPIO_10_ID != 0)
void GPIO_Pin10_IRQHandler(void)
{
  GPIO_IRQHandler(10);
}
#endif

#if (RTE_GPIO_11_ID != 0)
void GPIO_Pin11_IRQHandler(void)
{
  GPIO_IRQHandler(11);
}
#endif

#if (RTE_GPIO_12_ID != 0)
void GPIO_Pin12_IRQHandler(void)
{
  GPIO_IRQHandler(12);
}
#endif

#if (RTE_GPIO_13_ID != 0)
void GPIO_Pin13_IRQHandler(void)
{
  GPIO_IRQHandler(13);
}
#endif

#if (RTE_GPIO_14_ID != 0)
void GPIO_Pin14_IRQHandler(void)
{
  GPIO_IRQHandler(14);
}
#endif

#if (RTE_GPIO_15_ID != 0)
void GPIO_Pin15_IRQHandler(void)
{
  GPIO_IRQHandler(15);
}
#endif

#if (RTE_GPIO_16_ID != 0)
void GPIO_Pin16_IRQHandler(void)
{
  GPIO_IRQHandler(16);
}
#endif

#if (RTE_GPIO_17_ID != 0)
void GPIO_Pin17_IRQHandler(void)
{
  GPIO_IRQHandler(17);
}
#endif

#if (RTE_GPIO_18_ID != 0)
void GPIO_Pin18_IRQHandler(void)
{
  GPIO_IRQHandler(18);
}
#endif

#if (RTE_GPIO_19_ID != 0)
void GPIO_Pin19_IRQHandler(void)
{
  GPIO_IRQHandler(19);
}
#endif

#if (RTE_GPIO_20_ID != 0)
void GPIO_Pin20_IRQHandler(void)
{
  GPIO_IRQHandler(20);
}
#endif

#if (RTE_GPIO_21_ID != 0)
void GPIO_Pin21_IRQHandler(void)
{
  GPIO_IRQHandler(21);
}
#endif

#if (RTE_GPIO_22_ID != 0)
void GPIO_Pin22_IRQHandler(void)
{
  GPIO_IRQHandler(22);
}
#endif

#if (RTE_GPIO_23_ID != 0)
void GPIO_Pin23_IRQHandler(void)
{
  GPIO_IRQHandler(23);
}
#endif

#if (RTE_GPIO_24_ID != 0)
void GPIO_Pin24_IRQHandler(void)
{
  GPIO_IRQHandler(24);
}
#endif

#if (RTE_GPIO_25_ID != 0)
void GPIO_Pin25_IRQHandler(void)
{
  GPIO_IRQHandler(25);
}
#endif

#if (RTE_GPIO_26_ID != 0)
void GPIO_Pin26_IRQHandler(void)
{
  GPIO_IRQHandler(26);
}
#endif

#if (RTE_GPIO_27_ID != 0)
void GPIO_Pin27_IRQHandler(void)
{
  GPIO_IRQHandler(27);
}
#endif

#if (RTE_GPIO_28_ID != 0)
void GPIO_Pin28_IRQHandler(void)
{
  GPIO_IRQHandler(28);
}
#endif

#if (RTE_GPIO_29_ID != 0)
void GPIO_Pin29_IRQHandler(void)
{
  GPIO_IRQHandler(29);
}
#endif

#if (RTE_GPIO_30_ID != 0)
void GPIO_Pin30_IRQHandler(void)
{
  GPIO_IRQHandler(30);
}
#endif

#if (RTE_GPIO_31_ID != 0)
void GPIO_Pin31_IRQHandler(void)
{
  GPIO_IRQHandler(31);
}
#endif

/* GPIO Driver Control Block */
TZ10XX_DRIVER_GPIO Driver_GPIO = {
	GPIO_GetVersion,
	GPIO_GetCapabilities,
	GPIO_Initialize,
	GPIO_Uninitialize,
	GPIO_PowerControl ,
	GPIO_Configure,
	GPIO_ReadPin,
	GPIO_Read,
	GPIO_WritePin,
	GPIO_Write
};
 
