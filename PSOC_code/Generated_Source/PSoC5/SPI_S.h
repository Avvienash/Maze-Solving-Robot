/*******************************************************************************
* File Name: SPI_S.h
* Version 2.70
*
* Description:
*  Contains the function prototypes, constants and register definition
*  of the SPI Slave Component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SPIS_SPI_S_H)
#define CY_SPIS_SPI_S_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */


/***************************************
*   Conditional Compilation Parameters
***************************************/

#define SPI_S_DATA_WIDTH                  (8u)
#define SPI_S_INTERNAL_TX_INT_ENABLED     (0u)
#define SPI_S_INTERNAL_RX_INT_ENABLED     (0u)
#define SPI_S_MODE_USE_ZERO               (1u)
#define SPI_S_BIDIRECTIONAL_MODE          (0u)
#define SPI_S_MODE                        (0u)

#define SPI_S_FIFO_SIZE                  (4u)
/* Internal interrupt handling */
#define SPI_S_TX_BUFFER_SIZE             (4u)
#define SPI_S_RX_BUFFER_SIZE             (4u)
#define SPI_S_INTERNAL_TX_INT_ENABLED    (0u)
#define SPI_S_INTERNAL_RX_INT_ENABLED    (0u)

#define SPI_S_TX_SOFTWARE_BUF_ENABLED    ((0u != SPI_S_INTERNAL_TX_INT_ENABLED) && \
                                                     (SPI_S_TX_BUFFER_SIZE > SPI_S_FIFO_SIZE))

#define SPI_S_RX_SOFTWARE_BUF_ENABLED    ((0u != SPI_S_INTERNAL_RX_INT_ENABLED) && \
                                                     (SPI_S_RX_BUFFER_SIZE > SPI_S_FIFO_SIZE))


/***************************************
*        Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct
{
    uint8 enableState;
    uint8 cntrPeriod;
} SPI_S_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void  SPI_S_Init(void) ;
void  SPI_S_Enable(void) ;
void  SPI_S_Start(void) ;
void  SPI_S_Stop(void) ;
void  SPI_S_EnableTxInt(void) ;
void  SPI_S_EnableRxInt(void) ;
void  SPI_S_DisableTxInt(void) ;
void  SPI_S_DisableRxInt(void) ;
void  SPI_S_SetTxInterruptMode(uint8 intSrc) ;
void  SPI_S_SetRxInterruptMode(uint8 intSrc) ;
uint8 SPI_S_ReadTxStatus(void) ;
uint8 SPI_S_ReadRxStatus(void) ;
void  SPI_S_WriteTxData(uint8 txData);

#if(SPI_S_MODE_USE_ZERO != 0u)
    void  SPI_S_WriteTxDataZero(uint8 txDataByte) \
                                              ;
#endif /* (SPI_S_MODE_USE_ZERO != 0u) */

uint8 SPI_S_ReadRxData(void) ;
uint8 SPI_S_GetRxBufferSize(void) ;
uint8 SPI_S_GetTxBufferSize(void) ;
void  SPI_S_ClearRxBuffer(void) ;
void  SPI_S_ClearTxBuffer(void) ;

#if (SPI_S_BIDIRECTIONAL_MODE != 0u)
    void  SPI_S_TxEnable(void) ;
    void  SPI_S_TxDisable(void) ;
#endif /* SPI_S_BIDIRECTIONAL_MODE != 0u */

void  SPI_S_PutArray(const uint8 buffer[], uint8 byteCount) ;
void  SPI_S_ClearFIFO(void) ;
void  SPI_S_Sleep(void) ;
void  SPI_S_Wakeup(void) ;
void  SPI_S_SaveConfig(void) ;
void  SPI_S_RestoreConfig(void) ;

/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_SPI_S) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    SPI_S_CyBtldrCommStart(void) CYSMALL ;
    void    SPI_S_CyBtldrCommStop(void) CYSMALL ;
    void    SPI_S_CyBtldrCommReset(void) CYSMALL ;
    cystatus SPI_S_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus SPI_S_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_SPI_S)
        #define CyBtldrCommStart    SPI_S_CyBtldrCommStart
        #define CyBtldrCommStop     SPI_S_CyBtldrCommStop
        #define CyBtldrCommReset    SPI_S_CyBtldrCommReset
        #define CyBtldrCommWrite    SPI_S_CyBtldrCommWrite
        #define CyBtldrCommRead     SPI_S_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_SPI_S) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define SPI_S_BYTE2BYTE_TIME_OUT (1u)

#endif /* (CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_SPI_S) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)) */


CY_ISR_PROTO(SPI_S_TX_ISR);
CY_ISR_PROTO(SPI_S_RX_ISR);

/* Macros for getting software status of SPIS Statusi Register */
#define SPI_S_GET_STATUS_TX(swTxSts) ( (uint8)(SPI_S_TX_STATUS_REG | \
                                                       ((swTxSts) & SPI_S_STS_CLR_ON_RD_BYTES_MASK)) )
#define SPI_S_GET_STATUS_RX(swRxSts) ( (uint8)(SPI_S_RX_STATUS_REG | \
                                                       ((swRxSts) & SPI_S_STS_CLR_ON_RD_BYTES_MASK)) )


/***************************************
*   Variable with external linkage
***************************************/

extern uint8 SPI_S_initVar;


/***************************************
*           API Constants
***************************************/

#define SPI_S_TX_ISR_NUMBER     ((uint8)SPI_S_TxInternalInterrupt__INTC_NUMBER)
#define SPI_S_RX_ISR_NUMBER     ((uint8)SPI_S_RxInternalInterrupt__INTC_NUMBER)
#define SPI_S_TX_ISR_PRIORITY   ((uint8)SPI_S_TxInternalInterrupt__INTC_PRIOR_NUM)
#define SPI_S_RX_ISR_PRIORITY   ((uint8)SPI_S_RxInternalInterrupt__INTC_PRIOR_NUM)


/***************************************
*    Initial Parameter Constants
***************************************/

#define SPI_S_INT_ON_SPI_DONE    (uint8)(0u << SPI_S_STS_SPI_DONE_SHIFT)
#define SPI_S_INT_ON_TX_EMPTY    (uint8)(0u << SPI_S_STS_TX_FIFO_EMPTY_SHIFT)
#define SPI_S_INT_ON_TX_NOT_FULL (uint8)(0u << SPI_S_STS_TX_FIFO_NOT_FULL_SHIFT)
#define SPI_S_INT_ON_BYTE_COMP   (uint8)(0u << SPI_S_STS_BYTE_COMPLETE_SHIFT)

#define SPI_S_TX_INIT_INTERRUPTS_MASK  (SPI_S_INT_ON_SPI_DONE | \
                                            SPI_S_INT_ON_TX_EMPTY | SPI_S_INT_ON_TX_NOT_FULL | \
                                            SPI_S_INT_ON_BYTE_COMP)

#define SPI_S_INT_ON_RX_EMPTY     (uint8)(0u << SPI_S_STS_RX_FIFO_EMPTY_SHIFT)
#define SPI_S_INT_ON_RX_NOT_EMPTY (uint8)(0u << SPI_S_STS_RX_FIFO_NOT_EMPTY_SHIFT)
#define SPI_S_INT_ON_RX_OVER      (uint8)(0u << SPI_S_STS_RX_FIFO_OVERRUN_SHIFT)
#define SPI_S_INT_ON_RX_FULL      (uint8)(0u << SPI_S_STS_RX_FIFO_FULL_SHIFT)

#define SPI_S_RX_INIT_INTERRUPTS_MASK (SPI_S_INT_ON_RX_EMPTY | \
                                            SPI_S_INT_ON_RX_NOT_EMPTY | SPI_S_INT_ON_RX_OVER | \
                                            SPI_S_INT_ON_RX_FULL)

#define SPI_S_BITCTR_INIT           (SPI_S_DATA_WIDTH - 1u)

#define SPI_S__MODE_00 0
#define SPI_S__MODE_01 1
#define SPI_S__MODE_10 2
#define SPI_S__MODE_11 3


#define SPI_S_TX_BUFFER_SIZE         (4u)
#define SPI_S_RX_BUFFER_SIZE         (4u)

/* Following definitions are for version Compatibility, they are obsolete.
*  Please do not use it in new projects
*/
#define SPI_S_INIT_INTERRUPTS_MASK  (SPI_S_INT_ON_SPI_DONE | SPI_S_INT_ON_TX_EMPTY | \
                                            SPI_S_INT_ON_TX_NOT_FULL | SPI_S_INT_ON_RX_EMPTY | \
                                            SPI_S_INT_ON_RX_NOT_EMPTY | SPI_S_INT_ON_RX_OVER | \
                                            SPI_S_INT_ON_BYTE_COMP)


/***************************************
*             Registers
***************************************/
#if(CY_PSOC3 || CY_PSOC5)
    #define SPI_S_TXDATA_ZERO_REG          (* (reg8  *) \
            SPI_S_BSPIS_sR8_Dp_u0__A0_REG)

    #define SPI_S_TXDATA_ZERO_PTR           (  (reg8  *) \
            SPI_S_BSPIS_sR8_Dp_u0__A0_REG)

    #define SPI_S_RXDATA_ZERO_REG           (* (reg8  *) \
            SPI_S_BSPIS_sR8_Dp_u0__A0_REG)

    #define SPI_S_RXDATA_ZERO_PTR           (  (reg8  *) \
            SPI_S_BSPIS_sR8_Dp_u0__A0_REG)

    #define SPI_S_TXDATA_REG                (* (reg8  *) \
            SPI_S_BSPIS_sR8_Dp_u0__F0_REG)

    #define SPI_S_TXDATA_PTR                (  (reg8  *) \
            SPI_S_BSPIS_sR8_Dp_u0__F0_REG)

    #define SPI_S_RXDATA_REG                (* (reg8  *) \
            SPI_S_BSPIS_sR8_Dp_u0__F1_REG)

    #define SPI_S_RXDATA_PTR                (  (reg8  *) \
            SPI_S_BSPIS_sR8_Dp_u0__F1_REG)
#else
    #if(SPI_S_DATA_WIDTH <= 8u) /* 8bit - SPIS */
        #define SPI_S_TXDATA_ZERO_REG           (* (reg8 *) \
                SPI_S_BSPIS_sR8_Dp_u0__A0_REG)

        #define SPI_S_TXDATA_ZERO_PTR           (  (reg8  *) \
                SPI_S_BSPIS_sR8_Dp_u0__A0_REG)

        #define SPI_S_RXDATA_ZERO_REG           (* (reg8  *) \
                SPI_S_BSPIS_sR8_Dp_u0__A0_REG)

        #define SPI_S_RXDATA_ZERO_PTR           (  (reg8 *) \
                SPI_S_BSPIS_sR8_Dp_u0__A0_REG)

        #define SPI_S_TXDATA_REG                (* (reg8  *) \
                SPI_S_BSPIS_sR8_Dp_u0__F0_REG)

        #define SPI_S_TXDATA_PTR                (  (reg8  *) \
                SPI_S_BSPIS_sR8_Dp_u0__F0_REG)

        #define SPI_S_RXDATA_REG                (* (reg8  *) \
                SPI_S_BSPIS_sR8_Dp_u0__F1_REG)

        #define SPI_S_RXDATA_PTR                (  (reg8  *) \
                SPI_S_BSPIS_sR8_Dp_u0__F1_REG)
    #else /* 16bit - SPIS */
        #define SPI_S_TXDATA_ZERO_REG           (* (reg16  *) \
                SPI_S_BSPIS_sR8_Dp_u0__16BIT_A0_REG)

        #define SPI_S_TXDATA_ZERO_PTR           (  (reg16  *) \
                SPI_S_BSPIS_sR8_Dp_u0__16BIT_A0_REG)

        #define SPI_S_RXDATA_ZERO_REG           (* (reg16  *) \
                SPI_S_BSPIS_sR8_Dp_u0__16BIT_A0_REG)

        #define SPI_S_RXDATA_ZERO_PTR           (  (reg16  *) \
                SPI_S_BSPIS_sR8_Dp_u0__16BIT_A0_REG)

        #define SPI_S_TXDATA_REG                (* (reg16  *) \
                SPI_S_BSPIS_sR8_Dp_u0__16BIT_F0_REG)

        #define SPI_S_TXDATA_PTR                (  (reg16  *) \
                SPI_S_BSPIS_sR8_Dp_u0__16BIT_F0_REG)

        #define SPI_S_RXDATA_REG                (* (reg16  *) \
                SPI_S_BSPIS_sR8_Dp_u0__16BIT_F1_REG)

        #define SPI_S_RXDATA_PTR                (  (reg16 *) \
                SPI_S_BSPIS_sR8_Dp_u0__16BIT_F1_REG)
    #endif /* (SPI_S_DATA_WIDTH <= 8u) */
#endif     /* (CY_PSOC3 || CY_PSOC5) */

#define SPI_S_TX_AUX_CONTROL_DP0_REG       (* (reg8 *) \
        SPI_S_BSPIS_sR8_Dp_u0__DP_AUX_CTL_REG)
#define SPI_S_TX_AUX_CONTROL_DP0_PTR       (  (reg8 *) \
        SPI_S_BSPIS_sR8_Dp_u0__DP_AUX_CTL_REG)

#define SPI_S_RX_AUX_CONTROL_DP0_REG       (* (reg8 *) \
        SPI_S_BSPIS_sR8_Dp_u0__DP_AUX_CTL_REG)
#define SPI_S_RX_AUX_CONTROL_DP0_PTR       (  (reg8 *) \
        SPI_S_BSPIS_sR8_Dp_u0__DP_AUX_CTL_REG)

#if(SPI_S_DATA_WIDTH > 8u)

    #define SPI_S_TX_AUX_CONTROL_DP1_REG   (* (reg8 *) \
            SPI_S_BSPIS_sR8_Dp_u1__DP_AUX_CTL_REG)
    #define SPI_S_TX_AUX_CONTROL_DP1_PTR   (  (reg8 *) \
            SPI_S_BSPIS_sR8_Dp_u1__DP_AUX_CTL_REG)

    #define SPI_S_RX_AUX_CONTROL_DP1_REG   (* (reg8 *) \
            SPI_S_BSPIS_sR8_Dp_u1__DP_AUX_CTL_REG)
    #define SPI_S_RX_AUX_CONTROL_DP1_PTR   (  (reg8 *) \
            SPI_S_BSPIS_sR8_Dp_u1__DP_AUX_CTL_REG)

#endif /* SPI_S_DATA_WIDTH > 8u */


#define SPI_S_COUNTER_PERIOD_REG    (* (reg8 *) \
        SPI_S_BSPIS_BitCounter__PERIOD_REG)
#define SPI_S_COUNTER_PERIOD_PTR    (  (reg8 *) \
        SPI_S_BSPIS_BitCounter__PERIOD_REG)

#define SPI_S_TX_STATUS_MASK_REG    (* (reg8 *) \
        SPI_S_BSPIS_TxStsReg__MASK_REG)
#define SPI_S_TX_STATUS_MASK_PTR    (  (reg8 *) \
        SPI_S_BSPIS_TxStsReg__MASK_REG)

#define SPI_S_RX_STATUS_MASK_REG    (* (reg8 *) \
        SPI_S_BSPIS_RxStsReg__MASK_REG)
#define SPI_S_RX_STATUS_MASK_PTR    (  (reg8 *) \
        SPI_S_BSPIS_RxStsReg__MASK_REG)

#define SPI_S_ONE_REG               (* (reg8 *) \
        SPI_S_BSPIS_SPISlave_dpCounter_u0__D1_REG)
#define SPI_S_ONE_PTR               (  (reg8 *) \
        SPI_S_BSPIS_dpCounter_u0__D1_REG)

#define SPI_S_TX_STATUS_REG         (* (reg8 *) \
        SPI_S_BSPIS_TxStsReg__STATUS_REG)
#define SPI_S_TX_STATUS_PTR         (  (reg8 *) \
        SPI_S_BSPIS__TxStsReg__STATUS_REG)

#define SPI_S_RX_STATUS_REG         (* (reg8 *) \
        SPI_S_BSPIS_RxStsReg__STATUS_REG)
#define SPI_S_RX_STATUS_PTR         (  (reg8 *) \
        SPI_S_BSPIS_RxStsReg__STATUS_REG)

#define SPI_S_COUNTER_CONTROL_REG   (* (reg8 *) \
        SPI_S_BSPIS_BitCounter__CONTROL_AUX_CTL_REG)
#define SPI_S_COUNTER_CONTROL_PTR   (  (reg8 *) \
        SPI_S_BSPIS_BitCounter__CONTROL_AUX_CTL_REG)

#define SPI_S_TX_STATUS_ACTL_REG    (* (reg8 *) \
        SPI_S_BSPIS_TxStsReg__STATUS_AUX_CTL_REG)
#define SPI_S_TX_STATUS_ACTL_PTR    (  (reg8 *) \
        SPI_S_TX_BSPIS_TxStsReg__STATUS_AUX_CTL_REG)

#define SPI_S_RX_STATUS_ACTL_REG    (* (reg8 *) \
        SPI_S_BSPIS_RxStsReg__STATUS_AUX_CTL_REG)
#define SPI_S_RX_STATUS_ACTL_PTR    (  (reg8 *) \
        SPI_S_RX_BSPIS_RxStsReg__STATUS_AUX_CTL_REG)

#if(SPI_S_BIDIRECTIONAL_MODE)

    #define SPI_S_CONTROL_REG       (* (reg8 *) \
   SPI_S_BSPIS_SyncCtl_CtrlReg__CONTROL_REG)
    #define SPI_S_CONTROL_PTR       (  (reg8 *) \
   SPI_S_BSPIS_SyncCtl_CtrlReg__CONTROL_REG)

#endif /* SPI_S_BIDIRECTIONAL_MODE */


/***************************************
*       Register Constants
***************************************/

/* Status Register Definitions */
#define SPI_S_STS_SPI_DONE_SHIFT             (0x00u)
#define SPI_S_STS_TX_FIFO_NOT_FULL_SHIFT     (0x01u)
#define SPI_S_STS_TX_FIFO_EMPTY_SHIFT        (0x02u)
#define SPI_S_STS_RX_FIFO_NOT_EMPTY_SHIFT    (0x03u)
#define SPI_S_STS_RX_FIFO_EMPTY_SHIFT        (0x04u)
#define SPI_S_STS_RX_FIFO_OVERRUN_SHIFT      (0x05u)
#define SPI_S_STS_RX_FIFO_FULL_SHIFT         (0x06u)
#define SPI_S_STS_BYTE_COMPLETE_SHIFT        (0x06u)

#define SPI_S_STS_SPI_DONE                   ((uint8)(0x01u << SPI_S_STS_SPI_DONE_SHIFT))
#define SPI_S_STS_TX_FIFO_EMPTY              ((uint8)(0x01u << SPI_S_STS_TX_FIFO_EMPTY_SHIFT))
#define SPI_S_STS_TX_FIFO_NOT_FULL           ((uint8)(0x01u << SPI_S_STS_TX_FIFO_NOT_FULL_SHIFT))
#define SPI_S_STS_RX_FIFO_EMPTY              ((uint8)(0x01u << SPI_S_STS_RX_FIFO_EMPTY_SHIFT))
#define SPI_S_STS_RX_FIFO_NOT_EMPTY          ((uint8)(0x01u << SPI_S_STS_RX_FIFO_NOT_EMPTY_SHIFT))
#define SPI_S_STS_RX_FIFO_OVERRUN            ((uint8)(0x01u << SPI_S_STS_RX_FIFO_OVERRUN_SHIFT))
#define SPI_S_STS_RX_FIFO_FULL               ((uint8)(0x01u << SPI_S_STS_RX_FIFO_FULL_SHIFT))
#define SPI_S_STS_BYTE_COMPLETE              ((uint8)(0x01u << SPI_S_STS_BYTE_COMPLETE_SHIFT))

#define SPI_S_STS_CLR_ON_RD_BYTES_MASK       (0x61u)

/* StatusI Register Interrupt Enable Control Bits */
/* As defined by the Register map for the AUX Control Register */
#define SPI_S_INT_ENABLE                     (0x10u)
#define SPI_S_TX_FIFO_CLR    (0x01u) /* F0 - TX FIFO */
#define SPI_S_RX_FIFO_CLR    (0x02u) /* F1 - RX FIFO */
#define SPI_S_FIFO_CLR       (SPI_S_TX_FIFO_CLR | SPI_S_RX_FIFO_CLR)

/* Bit Counter (7-bit) Control Register Bit Definitions */
/* As defined by the Register map for the AUX Control Register */
#define SPI_S_CNTR_ENABLE                    (0x20u)

/* Bi-Directional mode control bit */
#define SPI_S_CTRL_TX_SIGNAL_EN              (0x01u)

/* Datapath Auxillary Control Register definitions */
#define SPI_S_AUX_CTRL_FIFO0_CLR             (0x00u)
#define SPI_S_AUX_CTRL_FIFO1_CLR             (0x00u)
#define SPI_S_AUX_CTRL_FIFO0_LVL             (0x00u)
#define SPI_S_AUX_CTRL_FIFO1_LVL             (0x00u)
#define SPI_S_STATUS_ACTL_INT_EN_MASK        (0x10u)

/* Component disabled */
#define SPI_S_DISABLED   (0u)

/***************************************
* The following code is DEPRECATED and 
* should not be used in new projects.
***************************************/

#define SPI_S_TXDATA_ZERO               (SPI_S_TXDATA_ZERO_REG)
#define SPI_S_TXDATA                    (SPI_S_TXDATA_REG)
#define SPI_S_RXDATA                    (SPI_S_RXDATA_REG)
#define SPI_S_MISO_AUX_CONTROLDP0       (SPI_S_MISO_AUX_CTRL_DP0_REG)
#define SPI_S_MOSI_AUX_CONTROLDP0       (SPI_S_MOSI_AUX_CTRL_DP0_REG)
#define SPI_S_TXBUFFERREAD              (SPI_S_txBufferRead)
#define SPI_S_TXBUFFERWRITE             (SPI_S_txBufferWrite)
#define SPI_S_RXBUFFERREAD              (SPI_S_rxBufferRead)
#define SPI_S_RXBUFFERWRITE             (SPI_S_rxBufferWrite)

#if(SPI_S_DATA_WIDTH > 8u)

    #define SPI_S_MISO_AUX_CONTROLDP1   (SPI_S_MISO_AUX_CTRL_DP1_REG)
    #define SPI_S_MOSI_AUX_CONTROLDP1   (SPI_S_MOSI_AUX_CTRL_DP0_REG)

#endif /* SPI_S_DATA_WIDTH > 8u */

#define SPI_S_COUNTER_PERIOD            (SPI_S_COUNTER_PERIOD_REG)
#define SPI_S_COUNTER_CONTROL           (SPI_S_COUNTER_CONTROL_REG)
#define SPI_S_ONE                       (SPI_S_ONE_REG)
#define SPI_S_STATUS                    (SPI_S_TX_STATUS_REG)
#define SPI_S_STATUS_MASK               (SPI_S_TX_STATUS_MASK_REG)
#define SPI_S_STATUS_ACTL               (SPI_S_TX_STATUS_ACTL_REG)

#define SPI_S_WriteByte      (SPI_S_WriteTxData)
#define SPI_S_ReadByte       (SPI_S_ReadRxData)
#define SPI_S_WriteByteZero  (SPI_S_WriteTxDataZero)
void  SPI_S_SetInterruptMode(uint8 intSrc) ;
uint8 SPI_S_ReadStatus(void) ;
void  SPI_S_EnableInt(void) ;
void  SPI_S_DisableInt(void) ;

#define SPI_S_STS_TX_BUF_NOT_FULL_SHIFT      (0x01u)
#define SPI_S_STS_TX_BUF_FULL_SHIFT          (0x02u)
#define SPI_S_STS_RX_BUF_NOT_EMPTY_SHIFT     (0x03u)
#define SPI_S_STS_RX_BUF_EMPTY_SHIFT         (0x04u)
#define SPI_S_STS_RX_BUF_OVERRUN_SHIFT       (0x05u)

#define SPI_S_STS_TX_BUF_NOT_FULL            ((uint8)(0x01u << SPI_S_STS_TX_BUF_NOT_FULL_SHIFT))
#define SPI_S_STS_TX_BUF_FULL                ((uint8)(0x01u << SPI_S_STS_TX_BUF_FULL_SHIFT))
#define SPI_S_STS_RX_BUF_NOT_EMPTY           ((uint8)(0x01u << SPI_S_STS_RX_BUF_NOT_EMPTY_SHIFT))
#define SPI_S_STS_RX_BUF_EMPTY               ((uint8)(0x01u << SPI_S_STS_RX_BUF_EMPTY_SHIFT))
#define SPI_S_STS_RX_BUF_OVERRUN             ((uint8)(0x01u << SPI_S_STS_RX_BUF_OVERRUN_SHIFT))

#define SPI_S_DataWidth                  (SPI_S_DATA_WIDTH)
#define SPI_S_InternalClockUsed          (SPI_S_INTERNAL_CLOCK)
#define SPI_S_InternalTxInterruptEnabled (SPI_S_INTERNAL_TX_INT_ENABLED)
#define SPI_S_InternalRxInterruptEnabled (SPI_S_INTERNAL_RX_INT_ENABLED)
#define SPI_S_ModeUseZero                (SPI_S_MODE_USE_ZERO)
#define SPI_S_BidirectionalMode          (SPI_S_BIDIRECTIONAL_MODE)
#define SPI_S_Mode                       (SPI_S_MODE)
#define SPI_S_DATAWIDHT                  (SPI_S_DATA_WIDTH)
#define SPI_S_InternalInterruptEnabled   (0u)

#define SPI_S_TXBUFFERSIZE   (SPI_S_TX_BUFFER_SIZE)
#define SPI_S_RXBUFFERSIZE   (SPI_S_RX_BUFFER_SIZE)

#define SPI_S_TXBUFFER       SPI_S_txBuffer
#define SPI_S_RXBUFFER       SPI_S_rxBuffer

#endif  /* CY_SPIS_SPI_S_H */

/* [] END OF FILE */
