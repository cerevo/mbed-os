/**
 * @file ADCC24_TZ10xx_internal.h
 * @brief a internal header file for TZ10xx ADCC24 driver
 * @note
 */

/*
 * COPYRIGHT (C) 2014-2017
 * TOSHIBA CORPORATION STORAGE & ELECTRONIC DEVICES SOLUTIONS COMPANY
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

#ifndef ADCC24_TZ10XX_INTERNAL_H
#define ADCC24_TZ10XX_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif


typedef struct {
    union {
        __IO uint32_t  CH_MODE;                    /*!< Channel mode setting register                                       */

        struct {
            __IO uint32_t  ChEn       :  1;        /*!< Channel enablingIt is chosen for the Analog input of this channel
                                                        whether it is AD translation . It is effective only at the time
                                                        of One-Time and Cyclic Scan mode. It is not concerned with the
                                                        set point of this bit at the time of Singe mode, but the channel
                                                        set up by [MODESEL].ChSel is chosen.0 : It is not Used.1 : It
                                                        is Used.                                                             */
            __IO uint32_t  DtFmt      :  1;        /*!< Conversion data storing formatThe format of the data stored
                                                        in FIFO is chosen.0 : Sign-less Integer1 : Integer with Sign          */
            __IO uint32_t  OfstEn     :  1;        /*!< Conversion data correction function0 : It Does Not Rectify.1
                                                        : It Rectifies.                                                       */
            __IO uint32_t  DMAEn      :  1;        /*!< DMA interface function0 : DMA Transfer Demand Disable1 : DMA
                                                        Transfer Demand Enabling                                              */
            __IO uint32_t  DownSpl    :  3;        /*!< Sampling infanticide000 : It Does Not Cull Out (Each Time Sampling).001
                                                        : It Samples Once to 2 Times.010 : It Samples Once to 3 Times.--110
                                                        : It Samples Once to 7 Times.111 : It Samples Once to 8 Times.       */
            __IO uint32_t  DMAReq     :  1;        /*!< DMA status[At the time of Read]0 : Nothing [ DMA Request ]1
                                                        : Under DMA Request[At the time of Write]0 : -1 : DMA Request
                                                        Cancellation                                                         */
            __IO uint32_t  WaterMark  :  4;        /*!< Watermark interruption number of sectionThe number of section
                                                        of FIFO which generates Watermark interruption is chosen.0000
                                                        : One Step0001 : Two Steps0010 : Three Steps0011 : Four Steps--1110
                                                        : 15 Steps1111 : 16 Steps                                            */
        } CH_MODE_b;                               /*!< BitSize                                                               */
    };

    union {
        __IO uint32_t  CH_INTEN;                   /*!< Channel interrupt control register                                  */

        struct {
            __IO uint32_t  ChEocEn    :  1;        /*!< The completion interruption of conversion0 : Disable1 : Enabling      */
            __IO uint32_t  WMIntEn    :  1;        /*!< Watermark interruption0 : Disable1 : Enabling                         */
            __IO uint32_t  FstDLostEn :  1;        /*!< 1st Data Lost interruption0 : Disable1 : Enabling                     */
            __IO uint32_t  LstDLostEn :  1;        /*!< Last Data Lost interruption0 : Disable1 : Enabling                    */
            __IO uint32_t  FIFOFullEn :  1;        /*!< FIFO Full interruption0 : Disable1 : Enabling                         */
            __IO uint32_t  FIFOurunEn :  1;        /*!< FIFO under run interruption0 : Disable1 : Enabling                    */
        } CH_INTEN_b;                              /*!< BitSize                                                               */
    };

    union {
        __IO uint32_t  CH_INTSTS;                  /*!< Channel interruption status register                                */

        struct {
            __IO uint32_t  ChEoc      :  1;        /*!< The completion interruption of conversion[At the time of Read]0
                                                        : With No Interruption Generating1 : Those with Interruption
                                                        Generating[At the time of Write]0 : -1 : Interruption Factor
                                                        Clearance                                                            */
            __IO uint32_t  WMInt      :  1;        /*!< Watermark interruption[At the time of Read]0 : With No Interruption
                                                        Generating1 : Those with Interruption Generating[At the time
                                                        of Write]0 : -1 : Interruption Factor Clearance                      */
            __IO uint32_t  FstDLost   :  1;        /*!< 1st Data Lost interruption[At the time of Read]0 : With No Interruption
                                                        Generating1 : Those with Interruption Generating[At the time
                                                        of Write]0 : -1 : Interruption Factor Clearance                      */
            __IO uint32_t  LstDLost   :  1;        /*!< Last Data Lost interruption[At the time of Read]0 : With No
                                                        Interruption Generating1 : Those with Interruption Generating[At
                                                        the time of Write]0 : -1 : Interruption Factor Clearance             */
            __IO uint32_t  FIFOFull   :  1;        /*!< FIFO Full interruption[At the time of Read]0 : With No Interruption
                                                        Generating1 : Those with Interruption Generating[At the time
                                                        of Write]0 : -1 : Interruption Factor Clearance                      */
            __IO uint32_t  FIFOurun   :  1;        /*!< FIFO under run interruption[At the time of Read]0 : With No
                                                        Interruption Generating1 : Those with Interruption Generating[At
                                                        the time of Write]0 : -1 : Interruption Factor Clearance             */
            } CH_INTSTS_b;                         /*!< BitSize                                                               */
    };

    union {
        __IO uint32_t  CH_OFFSET;                  /*!< Channel correction offset data                                      */

        struct {
            __IO uint32_t  Offset     :  8;        /*!< Offset data for conversion result compensationWhen it is made
                                                        a setup which uses a compensation function, as for the data
                                                        stored in FIFO, the value of this register will be added to
                                                        a conversion result.A most significant bit becomes a sign (a
                                                        sign extend is carried out and operation with conversion result
                                                        data is performed.).A Limiti value is stored in FIFO when a
                                                        calculated result exceeds the maximum and minimum of the conversion
                                                        range.                                                               */
        } CH_OFFSET_b;                             /*!< BitSize                                                               */
    };

    union {
        __I  uint32_t  CH_FIFOSTS;                 /*!< Channel FIFO status                                                 */

        struct {
            __I  uint32_t  DataNum    :  5;        /*!< The number of data in FIFO0~16 word                                   */
        } CH_FIFOSTS_b;                            /*!< BitSize                                                               */
    };
    __I  uint32_t  RESERVED2;

    union {
        __IO uint32_t  CH_AFEMODE;                 /*!< Channel AFE mode setting register                                   */

        struct {
               uint32_t               :  2;
            __IO uint32_t  Mode       : 10;        /*!< AFE mode setupConfigurable AFE in the preceding paragraph of
                                                        deltasigmaADC is set up.It is T.B.D for details.* Connect with
                                                        a macroscopic 24bit deltasigmaADCCNT_MODE [11:2] signal.             */
        } CH_AFEMODE_b;                            /*!< BitSize                                                               */
    };
    __I  uint32_t  RESERVED3;
} adcc24_ch_Type;

/* Data type */
typedef struct {
    union {
        __I  uint32_t  CH_DATA[16];                /*!< Channel conversion result data register* It is accessed by
                                                        the FIFO with every same address although there is a 0x800 to
                                                        0x83-f domain for 64 bytes.Please be sure to lead at 32 bits.8
                                                        bits/16bit read is also updated by the data of the next stage
                                                        for FIFO structure.                                                  */

        struct {
            __I  uint32_t  CmpData    : 24;        /*!< Conversion data FIFO of CH0                                           */
            __I  uint32_t  CmpDataSign:  8;        /*!< CmpDataSign                                                           */
        } CH_DATA_b[16];                           /*!< BitSize                                                               */
    };
} adcc24_data_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif


#define adcc24_ch0_BASE    (adcc24_ch_Type*)0x40049100UL    /* Channel0 mode config */
#define adcc24_ch1_BASE    (adcc24_ch_Type*)0x40049120UL    /* Channel1 mode config */
#define adcc24_ch2_BASE    (adcc24_ch_Type*)0x40049140UL    /* Channel2 mode comfig */

#define ADCC24_ch_BASE     {adcc24_ch0_BASE, adcc24_ch1_BASE, adcc24_ch2_BASE}

#define adcc24_data0_BASE  (adcc24_data_Type*)0x40049800UL  /* Channel0 data */
#define adcc24_data1_BASE  (adcc24_data_Type*)0x40049840UL  /* Channel1 data */
#define adcc24_data2_BASE  (adcc24_data_Type*)0x40049880UL  /* Channel2 data */

#define ADCC24_data_BASE   {adcc24_data0_BASE, adcc24_data1_BASE, adcc24_data2_BASE}


#ifdef __cplusplus
}
#endif

#endif /* ADCC24_TZ10XX_INTERNAL_H */

