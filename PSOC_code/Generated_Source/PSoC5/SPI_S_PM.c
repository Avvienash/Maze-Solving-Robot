/*******************************************************************************
* File Name: SPI_S_PM.c
* Version 2.70
*
* Description:
*  This file contains the setup, control and status commands to support
*  component operations in low power mode.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "SPI_S_PVT.h"

static SPI_S_BACKUP_STRUCT SPI_S_backup = 
{
    SPI_S_DISABLED,
    SPI_S_BITCTR_INIT,
};


/*******************************************************************************
* Function Name: SPI_S_SaveConfig
********************************************************************************
*
* Summary:
*  Empty function. Included for consistency with other components.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void SPI_S_SaveConfig(void) 
{

}


/*******************************************************************************
* Function Name: SPI_S_RestoreConfig
********************************************************************************
*
* Summary:
*  Empty function. Included for consistency with other components.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void SPI_S_RestoreConfig(void) 
{

}


/*******************************************************************************
* Function Name: SPI_S_Sleep
********************************************************************************
*
* Summary:
*  Prepare SPI Slave Component goes to sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  SPI_S_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void SPI_S_Sleep(void) 
{
    /* Save components enable state */
    if ((SPI_S_TX_STATUS_ACTL_REG & SPI_S_INT_ENABLE) != 0u)
    {
        SPI_S_backup.enableState = 1u;
    }
    else /* Components block is disabled */
    {
        SPI_S_backup.enableState = 0u;
    }

    SPI_S_Stop();

}


/*******************************************************************************
* Function Name: SPI_S_Wakeup
********************************************************************************
*
* Summary:
*  Prepare SPIM Component to wake up.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  SPI_S_backup - used when non-retention registers are restored.
*  SPI_S_txBufferWrite - modified every function call - resets to
*  zero.
*  SPI_S_txBufferRead - modified every function call - resets to
*  zero.
*  SPI_S_rxBufferWrite - modified every function call - resets to
*  zero.
*  SPI_S_rxBufferRead - modified every function call - resets to
*  zero.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void SPI_S_Wakeup(void) 
{
    #if (SPI_S_TX_SOFTWARE_BUF_ENABLED)
        SPI_S_txBufferFull = 0u;
        SPI_S_txBufferRead = 0u;
        SPI_S_txBufferWrite = 0u;
    #endif /* SPI_S_TX_SOFTWARE_BUF_ENABLED */

    #if (SPI_S_RX_SOFTWARE_BUF_ENABLED)
        SPI_S_rxBufferFull = 0u;
        SPI_S_rxBufferRead = 0u;
        SPI_S_rxBufferWrite = 0u;
    #endif /* SPI_S_RX_SOFTWARE_BUF_ENABLED */

    SPI_S_ClearFIFO();

    /* Restore components block enable state */
    if (SPI_S_backup.enableState != 0u)
    {
         /* Components block was enabled */
         SPI_S_Enable();
    } /* Do nothing if components block was disabled */
}


/* [] END OF FILE */
