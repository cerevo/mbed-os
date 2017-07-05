/**
 * @file NOR_TZ10xx.c
 * @brief a header file for TZ10xx NOR driver
 * @date $Date:: 2015-02-06 17:06:26 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */
#include "Driver_NOR.h"
#include "RTE_Device.h"
#include "TZ10xx.h"
#include "NOR_TZ10xx.h"
#include "PMU_TZ10xx.h"

#if !defined( RTE_SPIC_DMA )
#define RTE_SPIC_DMA 0
#endif

#define ARM_NOR_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 22)   /* driver version */

/* SRAM address */
#define SRAM_REMAP0_CODE_START    0x10000000
#define SRAM_REMAP0_CODE_END        0x10040000
#define SRAM_REMAP0_DATA_START    0x20000000
#define SRAM_REMAP0_DATA_END        0x20008000
#define SRAM_REMAP1_CODE_START    0x10000000
#define SRAM_REMAP1_CODE_END        0x10038000
#define SRAM_REMAP1_DATA_START    0x20000000
#define SRAM_REMAP1_DATA_END        0x20010000

/* NOR Flash commands */
#define NOR_CMD_QUAD_PAGE_PROGRAM     0x00000032
#define NOR_CMD_PAGE_PROGRAM          		0x00000002
#define NOR_CMD_READ_STATUS1        				 0x00000005
#define NOR_CMD_READ_STATUS2        				 0x00000035
#define NOR_CMD_WRITE_STATUS          			0x00000001
#define NOR_CMD_WRITE_ENABLE     						0x00000006
#define NOR_CMD_WRITE_DISABLE                0x00000004
#define NOR_CMD_CHIP_ERASE      							0x000000C7
#define NOR_CMD_SECTOR_ERASE    						0x00000020
#define NOR_CMD_FAST_READ_QUAD      			0x0000006B
#define NOR_CMD_FAST_READ         						0x0000000B
#define NOR_CMD_ERASE_SECURITY_REG		 		0x00000044
#define NOR_CMD_PROGRAM_SECURITY_REG	0x00000042
#define NOR_CMD_READ_SECURITY_REG					0x00000048
#define NOR_CMD_READ_DEVICE_ID    			  		0x00000090
#define NOR_CMD_READ_UNIQUE_ID    					0x0000004B

/* NOR Flash security registser */
#define NOR_SECURITY_SIZE 			0x100  
#define NOR_SECURITY_N			 			4  

/* NOR Flash status registser bit */
#define NOR_B_BUSY 			0x0000  
#define NOR_B_WEL  			0x0001  
#define NOR_B_BP0					0x0002 
#define NOR_B_BP1					0x0003
#define NOR_B_BP2					0x0004 
#define NOR_B_TB					0x0005  
#define NOR_B_SEC				0x0006  
#define NOR_B_QE					0x0009  
#define NOR_B_CMP				0x000e 

#define NOR_STS_BUSY(A)   ((A>>NOR_B_BUSY)&0x01) //BUSY of NOR status register1
#define NOR_STS_WEL(A)   	((A>>NOR_B_WEL)&0x01) //BUSY of NOR status register1
#define NOR_STS_QE(A)			((A>>(NOR_B_QE-8))&0x01)  //QE of NOR status register2
#define NOR_VAL_WP(A, B, C, D, E)  	( (A<<NOR_B_CMP) \
																											| (B<<NOR_B_SEC)	\
																											| (C<<NOR_B_BP2)	\
																											| (D<<NOR_B_BP1)  \
																											| (E<<NOR_B_BP0)	)

#define NOR_SLEEP 				  600 //cycle
#define NOR_SLEEP_LOOP 75 //NOR_SLEEP/8cycle

#define NOR_ACT_IDLE   					0 // not execute
#define NOR_ACT_EXEC 					0x01 // execute


#if RTE_SPIC_DMA
#include "SDMAC_TZ10xx.h"

 /* NOR event flag */
typedef enum _NOR_EVENT {
  NOR_EVENT_ERR = 0,     
  NOR_EVENT_END,  
	NOR_EVENT_TD,     
	NOR_EVENT_SLP,     
  NOR_EVENT_WE,  
  NOR_EVENT_RS,     
  NOR_EVENT_PP,     
  NOR_EVENT_FR,  
} NOR_EVENT;
#endif

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_NOR_API_VERSION,
  ARM_NOR_DRV_VERSION
};

/* Sector Information */
static ARM_NOR_SECTOR ARM_NOR_Sector_(NOR_FLASH_NAME)[NOR_FLASH_SECTOR_NUM] = {
  NOR_FLASH_SECTORS
};

/* Device Information */
ARM_NOR_DEVICE ARM_NOR_Device0 = {
  NOR_FLASH_SIZE,
  NOR_FLASH_SECTOR_NUM,
  ARM_NOR_Sector_(NOR_FLASH_NAME),
  NOR_FLASH_EVAL
};

/* NOR Initialize status */
typedef enum _NOR_INIT_STATUS {
  NOR_UN_INITIALIZED    = 0,     
  NOR_INITIALIZED= 1,  
} NOR_INIT_STATUS;

/* NOR Quad Enable Information */
typedef enum _NOR_QE {
  NOR_QUAD_DISABLE    = 0,     
  NOR_QUAD_ENABLE= 1,  
} NOR_QE;

/* NOR SRAM Remap Information */
typedef enum _NOR_REMAP {
  NOR_REMAP0   = 0,     
  NOR_REMAP1= 1,  
} NOR_REMAP;

/* NOR Information  */
typedef struct {
	NOR_INIT_STATUS           init;    // Initialize State
	volatile ARM_POWER_STATE         power;                // Power mode
	NOR_QE                         qe;         // Quad Enable
	NOR_REMAP                   remap;    // SRAM Remap0/1
} NOR_INFO;

 /* NOR Transfer flags  */
typedef struct {
	volatile uint32_t cnt;
#if RTE_SPIC_DMA
  int32_t  ch;
  uint32_t ctl;
	volatile uint32_t src;
	volatile uint32_t dst;
	volatile uint16_t len;
	volatile NOR_EVENT         event;
#endif
	volatile uint8_t  sqe;
	volatile uint8_t act;
	uint8_t erase_chip;
} NOR_TRANS;

/* NOR Resources definition */
typedef struct {
  NOR_INFO         *info;                 //< NOR information
  NOR_TRANS      *trans;                 //< NOR Transfer flags
} const NOR_RESOURCES;

static NOR_INFO nor_info = {
	NOR_UN_INITIALIZED,
	ARM_POWER_OFF,
	NOR_QUAD_DISABLE,
	NOR_REMAP0
};

static NOR_TRANS nor_trans;

static NOR_RESOURCES nor_res = {	
	&nor_info,
	&nor_trans
};

#if RTE_SPIC_DMA
extern TZ10XX_DRIVER_SDMAC Driver_SDMAC;

/**
  \fn          static void NOR_DmaHandler (uint32_t ch, uint32_t event)
  \brief       SDMAC Interrupt handler.
	\param[in]   ch   channel 
	\param[in]   event   result of transfer  
  \return      void 
*/
static void NOR_DmaHandler(uint32_t ch, uint32_t event)
{
	if((SDMAC_EVENT_TRANSFER != event) || (ARM_POWER_FULL != nor_res.info->power)) {
		nor_res.trans->event = NOR_EVENT_ERR;	

	} else if(NOR_EVENT_FR == nor_res.trans->event) {
		if(0 >= nor_res.trans->cnt) {
			nor_res.trans->event = NOR_EVENT_END;
		} else {	
			uint32_t src = nor_res.trans->src;
			uint32_t len = nor_res.trans->cnt;
			if(0x100 < len) len = 0x100;
			nor_res.trans->len = (uint16_t)len;
			/* FastRead */
			spic->IntEn_b.PrgRdEndEn = 1;
			spic->PrgAccCtrl = ((len-1)<<24) | 0x00040230;
			spic->PrimaryBuffer0 = ((src&0xff00) << 16) | (src&0xff0000) | ((src&0xff000000) >> 16) | (src&0xff);
			spic->PrgStart_b.Start = 1;
			nor_res.trans->cnt -= len;
			nor_res.trans->src += len<<8;
			spic->PrimaryBuffer1_b.PriBuf7 = (uint8_t) (src&0xff); // for debug
		}
		
	} else {
		/* ReadStatus */
		spic->IntEn_b.PrgRdEndEn = 1;
		spic->PrgBufIOCtrl_b.SecQuadIOEn = 0;
		spic->PrgAccCtrl = 0x00000230;
		spic->PrimaryBuffer0 = NOR_CMD_READ_STATUS1;
		spic->PrgStart_b.Start = 1;
	}
}

/**
  \fn          static void NOR_SetNVICEnable(void)
  \brief       set NVIC enable.
\return      void
*/
static void NOR_SetNVICEnable(void)
{
	NVIC_ClearPendingIRQ(SPIC_IRQn);
	NVIC_EnableIRQ(SPIC_IRQn);
	return;
}

/**
  \fn          static void NOR_SetNVICDisable(void)
  \brief       set NVIC disable.
\return      void
*/
static void NOR_SetNVICDisable(void)
{
	NVIC_DisableIRQ(SPIC_IRQn);
	NVIC_ClearPendingIRQ(SPIC_IRQn);
	return;
}
#endif

/**
  \fn          static void NOR_Sleep (void)
  \brief       wait for NOR_SLEEP cycle.
\return      \ref void
*/
static void NOR_Sleep(void)
{
 int32_t t = NOR_SLEEP_LOOP; 
  while (t-- > 0) {
    __NOP();
    __NOP();
    __NOP();
    __NOP();
  }
}

/**
  \fn          static void NOR_Sleep9us (void)
  \brief       wait for 9us.
\return      \ref void
*/
static void NOR_Sleep9us(void)
{
	int32_t t = 9 * SystemCoreClock / 1000000; 
	while (t > 0) {
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		t -= 8;
	}
}
/**
  \fn          static uint32_t NOR_Poling (volatile int32_t *time, int32_t itvl, uint8_t cmd)
  \brief       wait until BUSY becomes to 0.
	\param[in]   time    time limit 
	\param[in]   itvl      count cycles for sleep 
	\param[in]   cmd     waiting command 
\return      \ref 0:end  1:timeout
*/
static uint32_t NOR_Poling(volatile int32_t *time, int32_t itvl, uint8_t cmd)
{
	volatile uint32_t tmp;

	spic->IntEn = 0x0;
	spic->IntStat = 0x00000007;
	spic->PrgBufIOCtrl_b.SecQuadIOEn = 0;
	spic->PrgBufIOCtrl_b.PriQuadIOEn = 0;
	spic->PrgOECtrl_b.PriOECtrlEn = 0;
	spic->PrgAccCtrl = 0x00000230;
	spic->PrimaryBuffer0 = NOR_CMD_READ_STATUS1;
	
	for( ; *time>0; *time-=itvl) {
		spic->PrgStart_b.Start = 1;
		while(1) {
			tmp  = spic->IntStat_b.PrgRdEnd;
			if (tmp) {
				spic->IntStat = 0x00000007;
				break; 
			}
			if(ARM_POWER_FULL != nor_res.info->power)  return 1;
		}
		if(NOR_CMD_WRITE_ENABLE != cmd) {
			tmp = NOR_STS_BUSY(spic->SecondaryBuffer_b[0].SecBuf0); //BUSY of StatusRegister1
		} else {
			tmp = (NOR_STS_WEL(spic->SecondaryBuffer_b[0].SecBuf0) + 1) & 0x1; //WEL of StatusRegister1
		}
		if(!tmp) {
			return 0;
		}
		NOR_Sleep();
		if(ARM_POWER_FULL != nor_res.info->power)  return 1;
	}
	return 1; 
}

/**
  \fn          static uint32_t NOR_CheckBusy(uint32_t mask)
  \brief       check SPIC status and BUSY .
	\param[in]   mask    bitmask for status 
\return      \ref 0:done  1:false
*/
static uint32_t NOR_CheckBusy(uint32_t mask)
{
	volatile int32_t time	= NOR_SLEEP<<1;
	volatile uint32_t tmp;

	nor_res.trans->act += NOR_ACT_EXEC;
	if(NOR_ACT_EXEC != nor_res.trans->act) {
		nor_res.trans->act -= NOR_ACT_EXEC;
		return 1 ;
	}
	
	for( ; time>0; time-=NOR_SLEEP) {
		tmp = spic->Status; 
		if(!(tmp&mask)) {
			break; 
		}
		NOR_Sleep();
	}
	if(NOR_Poling( &time, NOR_SLEEP, NOR_CMD_FAST_READ)) {
		nor_res.trans->act -= NOR_ACT_EXEC;
		return 1;
	} else {
		return 0;
	}		
}

/**
  \fn          static ARM_NOR_STATUS NOR_Idle(ARM_NOR_STATUS rtn)
  \brief       return with ending instruction.
	\param[in]   rtn    return value 
\return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS NOR_Idle(ARM_NOR_STATUS rtn)
{
	nor_res.trans->erase_chip = NOR_ACT_IDLE;
	nor_res.trans->act -= NOR_ACT_EXEC;
	return rtn; 
}

/**
  \fn          static uint32_t NOR_ExecCommand (uint32_t cmd)
  \brief       execute NOR Flash instruction.
	\param[in]   cmd    instruction 
\return      \ref 0:done  1:false
*/
static uint32_t NOR_ExecCommand(uint32_t cmd )
{
  volatile uint32_t tmp = 0;
	uint32_t mask = 0x2; 
 int32_t time	= 0x7fffffff;
	int32_t itvl = 0;
	
	switch(cmd & 0xff) {
		case  NOR_CMD_READ_STATUS1:       				 //0x00000005
		case  NOR_CMD_READ_STATUS2:      				// 0x00000035
			spic->PrgAccCtrl = 0x00000230;
			mask = 0x1; 
			break;
		case  NOR_CMD_WRITE_STATUS:         			//0x00000001
			NOR_Sleep9us();
			spic->PrgAccCtrl = 0x00020310;
			break;
		case  NOR_CMD_CHIP_ERASE:    							//0x000000C7
			NOR_Sleep9us();
			/* fall through */
		case  NOR_CMD_WRITE_ENABLE:    						//0x00000006
		case  NOR_CMD_WRITE_DISABLE:             // 0x00000004
			spic->PrgAccCtrl = 0x00000310;
			break;
		case  NOR_CMD_SECTOR_ERASE:   						//0x00000020	
		case  NOR_CMD_ERASE_SECURITY_REG:		 		//0x00000044
			NOR_Sleep9us();
			spic->PrgAccCtrl = 0x00030310;
			break;
		case  NOR_CMD_READ_DEVICE_ID:      				// 0x00000090
			spic->PrgAccCtrl = 0x01030230;
			mask = 0x1; 
			break;
		case  NOR_CMD_READ_UNIQUE_ID:      				// 0x0000004B
			spic->PrgAccCtrl = 0x07040230;
			mask = 0x1; 
			break;
		default:
			return 1; // Undefined Command
	}

	spic->IntEn = 0x0;
	spic->IntStat = 0x00000007;
	spic->PrgBufIOCtrl_b.SecQuadIOEn = 0;
	spic->PrgBufIOCtrl_b.PriQuadIOEn = 0;
	spic->PrgOECtrl_b.PriOECtrlEn = 0;
  spic->PrimaryBuffer0 = cmd;
  spic->PrgStart_b.Start = 1;
 
	for( ; time>0; time-=itvl) {
		tmp  = spic->IntStat;
    if (tmp & mask) {
			spic->IntStat = 0x00000007;
			break; 
    }
		NOR_Sleep();
		if(ARM_POWER_FULL != nor_res.info->power)  return 1;
	}
	
	if(1 != mask) {
		if(NOR_Poling( &time, itvl, (cmd&0xff))) return 1; 
	}
	if((NOR_CMD_READ_STATUS1 != (cmd&0xff)) && (NOR_CMD_READ_STATUS2 != (cmd&0xff))) {
		spic->PrimaryBuffer1_b.PriBuf7 = cmd&0xff;
	}
	return 0; 
}

/**
  \fn          static void NOR_ResetOff(void)
  \brief       Reset off NOR.
\return      void
*/
static void NOR_ResetOff(void)
{
	Driver_PMU.EnableModule(PMU_MODULE_SPIC, 1);
	return;
}

/**
  \fn          static void NOR_ResetOn(void)
  \brief       Reset on NOR.
\return      void
*/
static void NOR_ResetOn(void)
{
	Driver_PMU.EnableModule(PMU_MODULE_SPIC, 0);
	return;
}

/**
  \fn          static ARM_NOR_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION ARM_NOR_GetVersion(void)
{
  return DriverVersion;
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_PowerControl (ARM_POWER_STATE  state)
  \brief       Controls NOR Interface Power.
  \param[in]   state    Power state
  \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_PowerControl (ARM_POWER_STATE  state)
{
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if(ARM_POWER_FULL == nor_res.info->power) {
		if(NOR_CheckBusy(0x10001)) return ARM_NOR_ERROR;
	} else {
		nor_res.trans->act = NOR_ACT_EXEC;
	} 
	
	switch (state) {
    case ARM_POWER_OFF:
     if(ARM_POWER_OFF != nor_res.info->power) {
					NOR_ResetOn();
			}
      break;
    case ARM_POWER_LOW:
      if(ARM_POWER_OFF == nor_res.info->power) {
					NOR_ResetOff();
			}
			break;
   case ARM_POWER_FULL:
       if(ARM_POWER_OFF == nor_res.info->power) {
					NOR_ResetOff();
			}
			if(ARM_POWER_FULL != nor_res.info->power) {	
				spic->DirAccCtrl_b.PollWIP = 1;
			}
			break;
 		default:
			return NOR_Idle(ARM_NOR_ERROR);
	}
	
	nor_res.info->power = state;
  return NOR_Idle(ARM_NOR_OK);
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_Initialize (uint32_t base_addr)
  \brief       Initialize NOR Interface.
	\param[in]   base_addr    starting address of NOR Flash
  \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_Initialize (uint32_t base_addr)
{
	volatile uint32_t tmp = base_addr;
	
	if (NOR_UN_INITIALIZED != nor_res.info->init) return ARM_NOR_ERROR;

	nor_res.info->init = NOR_INITIALIZED;
	NOR_ResetOn();
#if RTE_SPIC_DMA
	NOR_SetNVICEnable(); 
	nor_res.trans->ch = -1;
#endif	
	nor_res.trans->act = NOR_ACT_IDLE;
	nor_res.trans->erase_chip =  NOR_ACT_IDLE;

	ARM_NOR_PowerControl(ARM_POWER_LOW);
	
	tmp = spic->PrgBufIOCtrl_b.PriQuadIOEn;
	nor_res.info->qe = (tmp)? NOR_QUAD_ENABLE : NOR_QUAD_DISABLE;
		
	tmp = spic->PrgSRAMTrSize_b.TrSize;
	nor_res.info->remap = (0x40000 > tmp)? NOR_REMAP1 : NOR_REMAP0;
	
 	return NOR_Idle(ARM_NOR_OK);
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_Uninitialize (void)
  \brief       De-initialize NOR Interface.
  \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_Uninitialize (void)
{
	volatile uint32_t tmp = nor_res.info->power;
	int32_t time_limit = nor_res.trans->act + nor_res.trans->erase_chip * 5;
	
	nor_res.info->power = ARM_POWER_OFF;

	if(ARM_POWER_FULL == tmp) {
	
		tmp = spic->Status_b.PrgAccBusy;
		if(tmp) {
			spic->IntStat_b.StopEnd = 1;
			spic->SpiStop_b.Stop = 1;
		}
	
		/* wait done */
		time_limit = (tmp | time_limit) * SystemCoreClock;
		for(;time_limit>0;time_limit-=NOR_SLEEP) {
				NOR_Sleep();
		}
	}
		
	NOR_ResetOn();
#if RTE_SPIC_DMA
	NOR_SetNVICDisable(); 
#endif

	nor_res.trans->erase_chip =  NOR_ACT_IDLE;
	nor_res.trans->act = NOR_ACT_IDLE;
	nor_res.info->init = NOR_UN_INITIALIZED;
	nor_res.info->qe = NOR_QUAD_DISABLE;
	nor_res.info->remap = NOR_REMAP0;
	
  return ARM_NOR_OK;
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_CheckRange (uint32_t addr, uint32_t data, uint32_t size, uint32_t flash_size)
  \brief       Write SRAM data to  NOR Flash .
	\param[in]   addr    Flash address 
	\param[in]   data    SRAM address 
 	\param[in]   size    data size
 	\param[in]   flash__size    Flash max size
 \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_CheckRange (uint32_t addr, uint32_t data, uint32_t size, uint32_t flash_size)
{
	uint32_t code_s = SRAM_REMAP0_CODE_START;
	uint32_t code_e = SRAM_REMAP0_CODE_END;
	uint32_t data_s = SRAM_REMAP0_DATA_START;
	uint32_t data_e = SRAM_REMAP0_DATA_END;
	uint32_t tmp = 0;
	
	if (flash_size <= addr) return ARM_NOR_ERROR;	

	if(NOR_REMAP1 == nor_res.info->remap) {
		code_s = SRAM_REMAP1_CODE_START;
		code_e = SRAM_REMAP1_CODE_END;
		data_s = SRAM_REMAP1_DATA_START;
		data_e = SRAM_REMAP1_DATA_END;
	}
	if (code_s > (uint32_t)data) {
		return ARM_NOR_ERROR;	
	} else if (code_e > (uint32_t)data) {
		tmp = code_e - (uint32_t)data;
	} else if (data_s > (uint32_t)data) {
		return ARM_NOR_ERROR;	
	} else if (data_e > (uint32_t)data) {
		tmp = data_e - (uint32_t)data;
	} else return ARM_NOR_ERROR;	

	if (0 == size) return ARM_NOR_ERROR;	
	if(tmp > (flash_size - addr)) {
		tmp = flash_size - addr;
	}
	if (tmp < size) return ARM_NOR_ERROR;	

	return ARM_NOR_OK;
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_WriteDataInternal (uint32_t dst, uint8_t *src, uint32_t len)
  \brief       Write SRAM to  NOR Flash .
	\param[in]   dst    (dst address << 8) | instruction
	\param[in]   src    pointer to src 
 	\param[in]   len    first transfer size
 \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_WriteDataInternal (uint32_t dst, uint8_t *src, uint32_t len)
{
	int32_t time	= 0x7fffffff;
	int32_t itvl = 0;
	
#if RTE_SPIC_DMA

	if((7 < RTE_SPIC_DMA_CH) || (0 >  RTE_SPIC_DMA_CH)) {
		nor_res.trans->ch  = Driver_SDMAC.ChannelAcquire(
			NOR_DmaHandler, SDMAC_SRC_MEMORY, SDMAC_DST_MEMORY,
			SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST);
	} else {
		if( Driver_SDMAC.ChannelInitialize( RTE_SPIC_DMA_CH,
			NOR_DmaHandler, SDMAC_SRC_MEMORY, SDMAC_DST_MEMORY,
			SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST, 0, 0)
			== SDMAC_OK) {
			nor_res.trans->ch  = RTE_SPIC_DMA_CH;
		}
	}
	if ( 0 > nor_res.trans->ch) return NOR_Idle(ARM_NOR_ERROR);
	
  nor_res.trans->ctl = SDMAC_CTL(SDMAC_WIDTH_1, SDMAC_INC_INCREMENT, SDMAC_MSIZE_16,
														SDMAC_WIDTH_4, SDMAC_INC_INCREMENT, SDMAC_MSIZE_4);

	nor_res.trans->dst = dst;
	nor_res.trans->src =  (uint32_t)src;
	nor_res.trans->len = (uint16_t)len;
	spic->IntEn = 0x0;
	spic->IntStat = 0x00000007;
	spic->PrgBufIOCtrl_b.PriQuadIOEn = 0;
	spic->PrgOECtrl_b.PriOECtrlEn = 0;

	/* TransferData */
	nor_res.trans->event = NOR_EVENT_TD;
	if(SDMAC_OK !=  Driver_SDMAC.ChannelTransferData( nor_res.trans->ch, 	
			(const uint8_t *)src,  (uint8_t *)(&spic->SecondaryBuffer[0]),
			len, nor_res.trans-> ctl)) {
			nor_res.trans->event = NOR_EVENT_ERR;
	}

	for( ; time>0; time-=itvl) {
		NOR_Sleep();
		if(ARM_POWER_FULL != nor_res.info->power) {
			nor_res.trans->event = NOR_EVENT_ERR;
		}
		if(NOR_EVENT_END == nor_res.trans->event || 	NOR_EVENT_ERR == nor_res.trans->event) {
			break; 
		}
		if(NOR_EVENT_SLP == nor_res.trans->event) {
				spic->IntEn_b.PrgRdEndEn = 1;
				spic->PrgStart_b.Start = 1;
		}
	}

	if((7 < RTE_SPIC_DMA_CH) || (0 >  RTE_SPIC_DMA_CH)) {
		Driver_SDMAC.ChannelRelease(nor_res.trans->ch);
	} else {
		Driver_SDMAC.ChannelUninitialize(nor_res.trans->ch);
	}
	nor_res.trans->ch = -1;
	if(NOR_EVENT_ERR == nor_res.trans->event)  return NOR_Idle(ARM_NOR_ERROR);
	
#else

	{	
		uint8_t *reg = (uint8_t *)(&spic->SecondaryBuffer[0]) + len;
		uint32_t  max, tmp;

		src += len;
	while(nor_res.trans->cnt > 0) {

		max = (240 < len) ? 240 : len;
		
		/* TransferData */
		for(tmp=1;tmp<max;tmp++) {
			*--reg = *--src;
		}
	
		if(NOR_Poling( &time, itvl, (dst&0xff))) return NOR_Idle(ARM_NOR_ERROR);  

		/* WriteEnable */
		spic->PrgAccCtrl = 0x00000310;
		spic->PrimaryBuffer0 = NOR_CMD_WRITE_ENABLE;
		spic->PrgStart_b.Start = 1;
		
		/* TransferData */
		for( ;tmp<len;tmp++) {
			*--reg = *--src;
		}
		
		for( ; time>0; time-=itvl) {
			tmp  = spic->IntStat_b.PrgWrEnd;
			if (tmp) {
				spic->IntStat = 0x00000007;
				break; 
			}
			NOR_Sleep();
			if(ARM_POWER_FULL != nor_res.info->power)  return NOR_Idle(ARM_NOR_ERROR); 
		}
		if(NOR_Poling( &time, itvl, NOR_CMD_WRITE_ENABLE)) return NOR_Idle(ARM_NOR_ERROR);  

		*--reg = *--src;

		/* ProgramPage */
		NOR_Sleep9us();
		spic->PrgAccCtrl = ((len-1)<<24) | 0x00030330;
		spic->PrgBufIOCtrl_b.SecQuadIOEn = nor_res.trans->sqe;
		spic->PrimaryBuffer0 = ((dst&0xff00) << 16) | (dst&0xff0000) | ((dst&0xff000000) >> 16) | (dst&0xff);
		spic->PrgStart_b.Start = 1;
		spic->PrimaryBuffer1_b.PriBuf7 = (uint8_t)(dst&0xff); // for debug

		src += len;
		nor_res.trans->cnt -= len;
		dst += len<<8;
		len = nor_res.trans->cnt;
		if(0x100 < len) len = 0x100;
		
		src += len;
		reg = (uint8_t *)(&spic->SecondaryBuffer[0]) + len;

		for( ; time>0; time-=itvl) {
			tmp  = spic->IntStat_b.PrgWrEnd;
			if (tmp) {
				spic->IntStat = 0x00000007;
				break; 
			}
			NOR_Sleep();
			if(ARM_POWER_FULL != nor_res.info->power)  return NOR_Idle(ARM_NOR_ERROR); 
		}
	}
	}

#endif

	if(NOR_Poling( &time, itvl, (dst&0xff))) return NOR_Idle(ARM_NOR_ERROR); 
	return NOR_Idle(ARM_NOR_OK);
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_WriteData (uint32_t addr, const uint8_t *data, uint32_t size)
  \brief       Write SRAM data to  NOR Flash .
	\param[in]   addr    dst address
	\param[in]   data    pointer to src 
 	\param[in]   size    data size
 \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_WriteData (uint32_t addr, const uint8_t *data, uint32_t size)
{
	uint32_t len;
	
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	if (ARM_NOR_OK != ARM_NOR_CheckRange(addr, (uint32_t)data, size, NOR_FLASH_SIZE)) return ARM_NOR_ERROR;	

	if(NOR_CheckBusy(0x00001)) return ARM_NOR_ERROR;

	len = 0x100 - (addr & 0xff);
	if(size < len) len = size;
	nor_res.trans->cnt = size;

	/* ReadStatus2 */
	if(NOR_ExecCommand(NOR_CMD_READ_STATUS2)) return NOR_Idle(ARM_NOR_ERROR);
	nor_res.trans->sqe = NOR_STS_QE(spic->SecondaryBuffer_b[0].SecBuf0);

	if((NOR_QUAD_ENABLE == nor_res.info->qe) && nor_res.trans->sqe) {
			addr = (addr<<8) |	NOR_CMD_QUAD_PAGE_PROGRAM;
	} else {
			addr = (addr<<8) |	NOR_CMD_PAGE_PROGRAM;
			nor_res.trans->sqe = 0;
	}

	return ARM_NOR_WriteDataInternal (addr, (uint8_t *)data, len);
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_EraseSector (uint32_t addr)
  \brief       Erase Sector in Flash Memory .
	\param[in]   addr    sector address
  \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_EraseSector (uint32_t addr)
{
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	addr &= ~(NOR_FLASH_SECTOR_SIZE - 1); //0x1000 align
	if ((NOR_FLASH_SIZE -  NOR_FLASH_SECTOR_SIZE) < addr) return ARM_NOR_ERROR;	
	
	if(NOR_CheckBusy(0x00001)) return ARM_NOR_ERROR;
 	
	/* WriteEnable */
	if(NOR_ExecCommand(NOR_CMD_WRITE_ENABLE)) return NOR_Idle(ARM_NOR_ERROR); 
	
	/* EraseSector */
	if(NOR_ExecCommand((((addr&0xff) << 24) | ((addr&0xff00) << 8) | ((addr&0xff0000) >> 8) | 
																		NOR_CMD_SECTOR_ERASE)))  return NOR_Idle(ARM_NOR_ERROR); 

	return NOR_Idle(ARM_NOR_OK); 
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_EraseChip (void)
  \brief       Erase all of NOR Flash .
  \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_EraseChip (void)
{
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	if(NOR_CheckBusy(0x00001)) return ARM_NOR_ERROR;
	nor_res.trans->erase_chip = NOR_ACT_EXEC;
	
	/* WriteEnable */
	if(NOR_ExecCommand(NOR_CMD_WRITE_ENABLE)) return NOR_Idle(ARM_NOR_ERROR); 
	
	/* EraseChip */
	if(NOR_ExecCommand(NOR_CMD_CHIP_ERASE))  return NOR_Idle(ARM_NOR_ERROR); 

	return NOR_Idle(ARM_NOR_OK); 
 }

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_WriteProtect (NOR_WRITE_PROTECT_SIZE size, NOR_WRITE_PROTECT_TB tb)
  \brief       Writeprotect on or off at a part of NOR Flash .
  \param[in]   size    writeprotect size
	\param[in]   tb    writeprotect from top or bottom
	\return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_WriteProtect (NOR_WRITE_PROTECT_SIZE size, NOR_WRITE_PROTECT_TB tb)
{
	uint32_t val ;
	uint32_t sts ;
	volatile uint8_t tmp;
	
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	/* set Status Register  (CMP, SEC, TB, BP2, BP1, BP0) */
	switch(size) {
		case NOR_0KB: /**< unprotect */
			val = NOR_VAL_WP(0, 0, 0, 0, 0); break;
		case NOR_4KB:    /**< protect 4Kbyte  */
			val = NOR_VAL_WP(0, 1, 0, 0, 1); break;
		case NOR_8KB:    /**< protect 8Kbyte  */
			val = NOR_VAL_WP(0, 1, 0, 1, 0); break;
		case NOR_16KB:    /**< protect 16Kbyte  */
			val = NOR_VAL_WP(0, 1, 0, 1, 1); break;
		case NOR_32KB:    /**< protect 32Kbyte  */
			val = NOR_VAL_WP(0, 1, 1, 0, 0); break;
		case NOR_64KB:    /**< protect 64Kbyte  */
			val = NOR_VAL_WP(0, 0, 0, 0, 1); break;
		case NOR_128KB:    /**< protect 128Kbyte  */
			val = NOR_VAL_WP(0, 0, 0, 1, 0); break;
		case NOR_256KB:    /**< protect 256Kbyte  */
			val = NOR_VAL_WP(0, 0, 0, 1, 1); break;
		case NOR_512KB:    /**< protect 512Kbyte  */
			val = NOR_VAL_WP(0, 0, 1, 0, 0); break;
		case NOR_768KB:    /**< protect 768Kbyte  */
			val = NOR_VAL_WP(1, 0, 0, 1, 1); break;
		case NOR_896KB:    /**< protect 896Kbyte  */
			val = NOR_VAL_WP(1, 0, 0, 1, 0); break;
		case NOR_960KB:    /**< protect 960Kbyte  */
			val = NOR_VAL_WP(1, 0, 0, 0, 1); break;
		case NOR_992KB:    /**< protect 992Kbyte  */
			val = NOR_VAL_WP(1, 1, 1, 0, 0); break;
		case NOR_1008KB:    /**< protect 1008Kbyte  */
			val = NOR_VAL_WP(1, 1, 0, 1, 1); break;
		case NOR_1016KB:    /**< protect 1016Kbyte  */
			val = NOR_VAL_WP(1, 1, 0, 1, 0); break;
		case NOR_1020KB:    /**< protect 1020Kbyte  */
			val = NOR_VAL_WP(1, 1, 0, 0, 1); break;
		case NOR_1024KB:      /**< Protect all */
			val = NOR_VAL_WP(0, 1, 1, 1, 1); break;
		default:
			return ARM_NOR_ERROR;
	}	
	
	switch(tb) {
		case NOR_TOP:
			if((val & (1<<NOR_B_CMP))) val |= (1<<NOR_B_TB) ;	break;
		case NOR_BOTTOM: 
			if(!(val & (1<<NOR_B_CMP))) val |= (1<<NOR_B_TB) ;	break;
		default:
			return ARM_NOR_ERROR;
	}
	
	if(NOR_CheckBusy(0x00001)) return ARM_NOR_ERROR;
 	
	/* ReadStatus */
	if(NOR_ExecCommand(NOR_CMD_READ_STATUS1)) return NOR_Idle(ARM_NOR_ERROR); 
	tmp = spic->SecondaryBuffer_b[0].SecBuf0 & 0xff;
	sts = tmp;
	if(NOR_ExecCommand(NOR_CMD_READ_STATUS2)) return NOR_Idle(ARM_NOR_ERROR); 
	tmp = (spic->SecondaryBuffer_b[0].SecBuf0 & 0xff);
	sts |= tmp << 8;

	/* WriteEnable */
	if(NOR_ExecCommand(NOR_CMD_WRITE_ENABLE)) return NOR_Idle(ARM_NOR_ERROR); 
	
	/* WriteStatus */
	sts &= ~(NOR_VAL_WP(1, 1, 1, 1, 1) | (1<<NOR_B_TB));
	sts |= val;
	if(NOR_ExecCommand(((sts<<8) | NOR_CMD_WRITE_STATUS)))  return NOR_Idle(ARM_NOR_ERROR); 

	return NOR_Idle(ARM_NOR_OK); 
}

#if RTE_SPIC_DMA
/**
  \fn          void SPIC_IRQHandler (void)
  \brief       Interrupt handler.
  \return      void 
*/
void SPIC_IRQHandler(void)
{
	volatile uint32_t tmp;
	uint32_t dst = nor_res.trans->dst;
	uint32_t len = nor_res.trans->len;

	spic->IntEn = 0x0;
	spic->IntStat = 0x00000007;
	if(ARM_POWER_FULL != nor_res.info->power)  {
		nor_res.trans->event = NOR_EVENT_ERR;
	}
	switch(nor_res.trans->event) {
		
		case NOR_EVENT_TD:
		case NOR_EVENT_SLP:
			tmp = NOR_STS_BUSY(spic->SecondaryBuffer_b[0].SecBuf0); //BUSY of StatusRegister1
			if(tmp) {
				nor_res.trans->event = NOR_EVENT_SLP;
			} else {
				/* WriteEnable */
				spic->IntEn_b.PrgWrEndEn = 1;
				spic->PrgAccCtrl = 0x00000310;
				spic->PrimaryBuffer0 = NOR_CMD_WRITE_ENABLE;
				spic->PrgStart_b.Start = 1;
				nor_res.trans->event = NOR_EVENT_WE;
			}
			break;
			
		case NOR_EVENT_WE:
			/* ReadStatus */
			spic->IntEn_b.PrgRdEndEn = 1;
			spic->PrgAccCtrl = 0x00000230;
			spic->PrimaryBuffer0 = NOR_CMD_READ_STATUS1;
			spic->PrgStart_b.Start = 1;
			nor_res.trans->event = NOR_EVENT_RS;
			break;
		
		case NOR_EVENT_RS:
			tmp = NOR_STS_WEL(spic->SecondaryBuffer_b[0].SecBuf0); //BUSY of StatusRegister1
			if(!tmp) {
				spic->IntEn_b.PrgRdEndEn = 1;
				spic->PrgStart_b.Start = 1;
			} else {	
				spic->SecondaryBuffer_b[0].SecBuf0 = *((uint8_t *)(nor_res.trans->src));
				/* ProgramPage */
				NOR_Sleep9us();
				spic->IntEn_b.PrgWrEndEn = 1;
				spic->PrgAccCtrl = ((len-1)<<24) | 0x00030330;
				spic->PrgBufIOCtrl_b.SecQuadIOEn = nor_res.trans->sqe;
				spic->PrimaryBuffer0 = ((dst&0xff00) << 16) | (dst&0xff0000) | ((dst&0xff000000) >> 16) | (dst&0xff);
				spic->PrgStart_b.Start = 1;
				spic->PrimaryBuffer1_b.PriBuf7 = (uint8_t) (dst&0xff); // for debug
				nor_res.trans->cnt -= len;
				nor_res.trans->src += len;
				nor_res.trans->dst += len<<8;
				len = nor_res.trans->cnt;
				if(0x100 < len) len = 0x100;
				nor_res.trans->len = (uint16_t)len;
				nor_res.trans->event = NOR_EVENT_PP;
			}
			break;
			
		case NOR_EVENT_PP:
			if(0 < nor_res.trans->cnt) {
				/* TransferData */
				nor_res.trans->event = NOR_EVENT_TD;
				if(SDMAC_OK !=  Driver_SDMAC.ChannelTransferData( nor_res.trans->ch, 
							(const uint8_t *)(nor_res.trans->src),  (uint8_t *)(&spic->SecondaryBuffer[0]),
							len, nor_res.trans-> ctl)) {
					nor_res.trans->event = NOR_EVENT_ERR;
				}			
			} else {
				nor_res.trans->event = NOR_EVENT_END;
			}
			break;
			
		case NOR_EVENT_FR:
			/* TransferData */
			if(SDMAC_OK !=  Driver_SDMAC.ChannelTransferData( nor_res.trans->ch, 
						(uint8_t *)(&spic->SecondaryBuffer[0]), (uint8_t *)(dst), 
						len, nor_res.trans-> ctl)) {
				nor_res.trans->event = NOR_EVENT_ERR;
			}			
			nor_res.trans->dst += len;
			break;
			
		default:
			nor_res.trans->event = NOR_EVENT_ERR;
			break;
	}
}
#endif

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_ReadDataInternal (uint32_t src, uint8_t *dst)
  \brief       Read  NOR Flash  to SRAM .
	\param[in]   src    (address << 8) | instruction
	\param[out]   dst    pointer to dst 
 \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_ReadDataInternal (uint32_t src, uint8_t *dst)
{
	int32_t time	= 0x7fffffff;
	int32_t itvl = 0;
	uint32_t len;
	
	spic->IntEn = 0x0;
	spic->IntStat = 0x00000007;
	spic->PrgBufIOCtrl_b.PriQuadIOEn = 0;
	spic->PrgOECtrl = 0x00000401;
	spic->PrgBufIOCtrl_b.SecQuadIOEn = 	nor_res.trans->sqe;

#if RTE_SPIC_DMA

	if((7 < RTE_SPIC_DMA_CH) || (0 >  RTE_SPIC_DMA_CH)) {
		nor_res.trans->ch  = Driver_SDMAC.ChannelAcquire(
			NOR_DmaHandler, SDMAC_SRC_MEMORY, SDMAC_DST_MEMORY,
			SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST);
	} else {
		if( Driver_SDMAC.ChannelInitialize( RTE_SPIC_DMA_CH,
			NOR_DmaHandler, SDMAC_SRC_MEMORY, SDMAC_DST_MEMORY,
			SDMAC_FIFO_MODE_UTILIZATION, SDMAC_PRIORITY_LOWEST, 0, 0)
			== SDMAC_OK) {
			nor_res.trans->ch  = RTE_SPIC_DMA_CH;
		}
	}
	if ( 0 > nor_res.trans->ch) return NOR_Idle(ARM_NOR_ERROR);
	
  nor_res.trans->ctl = SDMAC_CTL(SDMAC_WIDTH_1, SDMAC_INC_INCREMENT, SDMAC_MSIZE_16,
														SDMAC_WIDTH_1, SDMAC_INC_INCREMENT, SDMAC_MSIZE_16);

	nor_res.trans->event = NOR_EVENT_FR;
	nor_res.trans->dst = (uint32_t)dst;
	len = nor_res.trans->cnt;
	if(0x100 < len) len = 0x100;
	nor_res.trans->len = (uint16_t)len;
	nor_res.trans->src = src + (len<<8);
	nor_res.trans->cnt -= len;
	/* FastRead */
	spic->IntEn_b.PrgRdEndEn = 1;
	spic->PrgAccCtrl = ((len-1)<<24) | 0x00040230;
	spic->PrimaryBuffer0 = ((src&0xff00) << 16) | (src&0xff0000) | ((src&0xff000000) >> 16) | (src&0xff);
	spic->PrgStart_b.Start = 1;
	spic->PrimaryBuffer1_b.PriBuf7 = (uint8_t) (src&0xff); // for debug

	for( ; time>0; time-=itvl) {
		NOR_Sleep();
		if(ARM_POWER_FULL != nor_res.info->power) {
			nor_res.trans->event = NOR_EVENT_ERR;
		}
		if(NOR_EVENT_END == nor_res.trans->event || 	NOR_EVENT_ERR == nor_res.trans->event) {
			break; 
		}
	}

	if((7 < RTE_SPIC_DMA_CH) || (0 >  RTE_SPIC_DMA_CH)) {
		Driver_SDMAC.ChannelRelease(nor_res.trans->ch);
	} else {
		Driver_SDMAC.ChannelUninitialize(nor_res.trans->ch);
	}
	nor_res.trans->ch = -1;
	if(NOR_EVENT_ERR == nor_res.trans->event)  return NOR_Idle(ARM_NOR_ERROR);
	
#else

	{	
		uint32_t tmp;
		uint8_t *reg = (uint8_t *)(&spic->SecondaryBuffer[0]);

	while(nor_res.trans->cnt > 0) {

		len = nor_res.trans->cnt;
		if(0x100 < len) len = 0x100;

		/* FastRead */
		spic->PrgAccCtrl = ((len-1)<<24) | 0x00040230;
		spic->PrimaryBuffer0 = ((src&0xff00) << 16) | (src&0xff0000) | ((src&0xff000000) >> 16) | (src&0xff);
		spic->PrgStart_b.Start = 1;
		spic->PrimaryBuffer1_b.PriBuf7 = (uint8_t)(src&0xff); // for debug

		nor_res.trans->cnt -= len;
		src += len<<8;
		
		reg = (uint8_t *)(&spic->SecondaryBuffer[0]);

		for( ; time>0; time-=itvl) {
			tmp  = spic->IntStat_b.PrgRdEnd;
			if (tmp) {
				spic->IntStat = 0x00000007;
				break; 
			}
			NOR_Sleep();
			if(ARM_POWER_FULL != nor_res.info->power)  return NOR_Idle(ARM_NOR_ERROR); 
		}
		
		/* TransferData */
		for(tmp=0;tmp<len;tmp++) {
			*dst++ = *reg++;
		}
		if(ARM_POWER_FULL != nor_res.info->power)  return NOR_Idle(ARM_NOR_ERROR); 

	}
	}

#endif

	return NOR_Idle(ARM_NOR_OK);
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_ReadData (uint32_t addr, uint8_t *data, uint32_t size)
  \brief       Read  NOR Flash data  to SRAM .
	\param[in]   addr    src address
	\param[out]   data    pointer to dst 
 	\param[in]   size    data size
 \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_ReadData (uint32_t addr, uint8_t *data, uint32_t size)
{
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	if (ARM_NOR_OK != ARM_NOR_CheckRange(addr, (uint32_t)data, size, NOR_FLASH_SIZE)) return ARM_NOR_ERROR;	

	if(NOR_CheckBusy(0x00001)) return ARM_NOR_ERROR;

	nor_res.trans->cnt = size;

	/* ReadStatus2 */
	if(NOR_ExecCommand(NOR_CMD_READ_STATUS2)) return NOR_Idle(ARM_NOR_ERROR);
	nor_res.trans->sqe = NOR_STS_QE(spic->SecondaryBuffer_b[0].SecBuf0);

	if((NOR_QUAD_ENABLE == nor_res.info->qe) && nor_res.trans->sqe) {
			addr = (addr<<8) |	NOR_CMD_FAST_READ_QUAD;
	} else {
			nor_res.trans->sqe = 0;
			addr = (addr<<8) |	NOR_CMD_FAST_READ;
	}
	
	return ARM_NOR_ReadDataInternal (addr, data);
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_ReadSecurityRegister (uint32_t addr, uint8_t *data, uint32_t size)
  \brief       Read  NOR Flash Secutiry Register to SRAM .
	\param[in]   addr    SecurityRegister address
	\param[out]   data    pointer to dst 
 	\param[in]   size    data size
 \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_ReadSecurityRegister (uint32_t addr, uint8_t *data, uint32_t size)
{
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	if (0xff < (addr&0xfff)) return ARM_NOR_ERROR;	
	if (NOR_SECURITY_N <= (addr>>12)) return ARM_NOR_ERROR;	

	if ((0x100 - (addr&0xfff)) < size) return ARM_NOR_ERROR;	

	if (ARM_NOR_OK != ARM_NOR_CheckRange((addr&0xff), (uint32_t)data, size, NOR_SECURITY_SIZE)) return ARM_NOR_ERROR;	

	if(NOR_CheckBusy(0x00001)) return ARM_NOR_ERROR;

	nor_res.trans->cnt = size;
	addr = (addr<<8) |  NOR_CMD_READ_SECURITY_REG;
	nor_res.trans->sqe = 0;

	return ARM_NOR_ReadDataInternal (addr, data);
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_WriteSecurityRegister (uint32_t addr, const uint8_t *data, uint32_t size)
  \brief       Write SRAM to  NOR Flash Secutiry Register .
	\param[in]   addr    SecurityRegister address
	\param[out]   data    pointer to src 
 	\param[in]   size    data size
 \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_WriteSecurityRegister (uint32_t addr, const uint8_t *data, uint32_t size)
{
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	if (0xff < (addr&0xfff)) return ARM_NOR_ERROR;	
	if (NOR_SECURITY_N <= (addr>>12)) return ARM_NOR_ERROR;	

	if ((0x100 - (addr&0xfff)) < size) return ARM_NOR_ERROR;	

	if (ARM_NOR_OK != ARM_NOR_CheckRange((addr&0xff), (uint32_t)data, size, NOR_SECURITY_SIZE)) return ARM_NOR_ERROR;	

	if(NOR_CheckBusy(0x00001)) return ARM_NOR_ERROR;

	nor_res.trans->cnt = size;
	addr = (addr<<8) |  NOR_CMD_PROGRAM_SECURITY_REG;
	nor_res.trans->sqe = 0;

	return ARM_NOR_WriteDataInternal (addr, (uint8_t *)data, size);
}

/**
  \fn          static ARM_NOR_STATUS ARM_NOR_EraseSecurityRegister (uint32_t addr)
  \brief       Erase Sector in Flash Memory .
	\param[in]   addr    security register address
  \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_EraseSecurityRegister (uint32_t addr)
{
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	if (addr&0xfff) return ARM_NOR_ERROR;	
	if (NOR_SECURITY_N <= (addr>>12)) return ARM_NOR_ERROR;	
	
	if(NOR_CheckBusy(0x00001)) return ARM_NOR_ERROR;
 	
	/* WriteEnable */
	if(NOR_ExecCommand(NOR_CMD_WRITE_ENABLE)) return NOR_Idle(ARM_NOR_ERROR); 
	
	/* EraseSecrityRegister */
	if(NOR_ExecCommand(((addr<< 8) |  
																NOR_CMD_ERASE_SECURITY_REG)))  return NOR_Idle(ARM_NOR_ERROR); 

	return NOR_Idle(ARM_NOR_OK); 
}

/**
	\fn          static ARM_NOR_STATUS ARM_NOR_GetID (uint8_t *manufacturer, uint8_t *device, uint32_t *upper_unique, uint32_t *lower_unique)
  \brief       get IDs of NOR Flash .
	\param[out]   manufacturer		manufacturer ID
	\param[out]   device		device ID
 	\param[out]   unique	  unique ID
  \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_GetID (uint8_t *manufacturer, uint8_t *device, uint8_t *unique)
{
	uint32_t i;
	
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	if(NOR_CheckBusy(0x00001)) return ARM_NOR_ERROR;
 	
	/* ReadManufactureAndDeviceID */
	if(NOR_ExecCommand(NOR_CMD_READ_DEVICE_ID)) return NOR_Idle(ARM_NOR_ERROR); 
	*manufacturer = spic->SecondaryBuffer_b[0].SecBuf0;
	*device = spic->SecondaryBuffer_b[0].SecBuf1;

	/* ReadUniqueID */
	if(NOR_ExecCommand(NOR_CMD_READ_UNIQUE_ID)) return NOR_Idle(ARM_NOR_ERROR); 
	for(i=0;i<2;i++) {
		*unique++ = spic->SecondaryBuffer_b[i].SecBuf0;
		*unique++ = spic->SecondaryBuffer_b[i].SecBuf1;
		*unique++ = spic->SecondaryBuffer_b[i].SecBuf2;
		*unique++ = spic->SecondaryBuffer_b[i].SecBuf3;
	}

	return NOR_Idle(ARM_NOR_OK); 
}

/**
	\fn          static ARM_NOR_STATUS ARM_NOR_GetQuad (uint32_t *qe)
  \brief       get quad status .
	\param[out]   qe    quad value
  \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_GetQuad (uint8_t *qe)
{
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	if(NOR_CheckBusy(0x00001)) return ARM_NOR_ERROR;
 	
	/* ReadStatus2 */
	if(NOR_ExecCommand(NOR_CMD_READ_STATUS2)) return NOR_Idle(ARM_NOR_ERROR);
	*qe = NOR_STS_QE(spic->SecondaryBuffer_b[0].SecBuf0);

	return NOR_Idle(ARM_NOR_OK); 
}

/**
	\fn          static ARM_NOR_STATUS ARM_NOR_SetQuad (uint32_t qe)
  \brief       set quad status .
	\param[in]   qe    quad value
  \return      \ref ARM_NOR_STATUS
*/
static ARM_NOR_STATUS ARM_NOR_SetQuad (uint8_t qe)
{
	uint32_t sts;
	volatile uint8_t tmp;
	
	if(NOR_UN_INITIALIZED == nor_res.info->init)	return ARM_NOR_ERROR;
	if (ARM_POWER_FULL != nor_res.info->power) return ARM_NOR_ERROR;

	if (1 < qe) return ARM_NOR_ERROR;	
	
	if(NOR_CheckBusy(0x10001)) return ARM_NOR_ERROR;
 	
	/* ReadStatus2 */
	if(NOR_ExecCommand(NOR_CMD_READ_STATUS2)) return NOR_Idle(ARM_NOR_ERROR);
	tmp =spic->SecondaryBuffer_b[0].SecBuf0 & 0xff;
	sts =tmp;
	
	if(NOR_STS_QE(sts) != qe) {  
		/* ReadStatus1 */
		if(NOR_ExecCommand(NOR_CMD_READ_STATUS1)) return NOR_Idle(ARM_NOR_ERROR);
		tmp = spic->SecondaryBuffer_b[0].SecBuf0 & 0xff;
		sts = (sts<<8) | tmp;
		/* WriteEnable */
		if(NOR_ExecCommand(NOR_CMD_WRITE_ENABLE)) return NOR_Idle(ARM_NOR_ERROR);
		/* WriteStatus */
		sts &= ~(1<<NOR_B_QE);
		sts |= qe<<NOR_B_QE;
		if((NOR_QUAD_ENABLE == nor_res.info->qe) && qe) {
				spic->DirCmdCtrl = 0xeb00300e;
		} else {
				spic->DirCmdCtrl = 0x0b001000;
		}
		if(NOR_ExecCommand(((sts<<8) | NOR_CMD_WRITE_STATUS)))  return NOR_Idle(ARM_NOR_ERROR);
	}	
	return NOR_Idle(ARM_NOR_OK); 
}

/* NOR Driver Control Block */
ARM_DRIVER_NOR ARM_Driver_NOR0 = {
	ARM_NOR_GetVersion,
	ARM_NOR_Initialize,
	ARM_NOR_Uninitialize,
	ARM_NOR_PowerControl ,
	ARM_NOR_ReadData,
	ARM_NOR_WriteData,
	ARM_NOR_EraseSector,
	ARM_NOR_EraseChip
};
 
/* NOR Driver Control Block ( not ARM) */
TZ10XX_DRIVER_NOR Driver_NOR0 = {
	ARM_NOR_GetVersion,
	ARM_NOR_Initialize,
	ARM_NOR_Uninitialize,
	ARM_NOR_PowerControl ,
	ARM_NOR_ReadData,
	ARM_NOR_WriteData,
	ARM_NOR_EraseSector,
	ARM_NOR_EraseChip,
	ARM_NOR_WriteProtect,
	ARM_NOR_ReadSecurityRegister,
	ARM_NOR_WriteSecurityRegister,
	ARM_NOR_EraseSecurityRegister,
	ARM_NOR_GetID,
	ARM_NOR_GetQuad,
	ARM_NOR_SetQuad
};
 
