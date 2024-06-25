/*******************************************************************************
* File Name: .h
* Version 2.70
*
* Description:
*  This private header file contains internal definitions for the SPIS
*  component. Do not use these definitions directly in your application.
*
* Note:
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SPIS_PVT_SPI_S_H)
#define CY_SPIS_PVT_SPI_S_H

#include "SPI_S.h"


/**********************************
*   Functions with external linkage
**********************************/


/**********************************
*   Variables with external linkage
**********************************/
extern volatile uint8 SPI_S_swStatusTx;
extern volatile uint8 SPI_S_swStatusRx;

#if (SPI_S_RX_SOFTWARE_BUF_ENABLED)

    extern volatile uint8 SPI_S_rxBuffer[SPI_S_RX_BUFFER_SIZE];
    extern volatile uint8 SPI_S_rxBufferRead;
    extern volatile uint8 SPI_S_rxBufferWrite;
    extern volatile uint8 SPI_S_rxBufferFull;

#endif /* SPI_S_RX_SOFTWARE_BUF_ENABLED */

#if (SPI_S_TX_SOFTWARE_BUF_ENABLED)

    extern volatile uint8 SPI_S_txBuffer[SPI_S_TX_BUFFER_SIZE];
    extern volatile uint8 SPI_S_txBufferRead;
    extern volatile uint8 SPI_S_txBufferWrite;
    extern volatile uint8 SPI_S_txBufferFull;

#endif /* SPI_S_TX_SOFTWARE_BUF_ENABLED */

#endif /* CY_SPIS_PVT_SPI_S_H */


/* [] END OF FILE */
