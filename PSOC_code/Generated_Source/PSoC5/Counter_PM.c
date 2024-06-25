/*******************************************************************************
* File Name: Counter_PM.c  
* Version 3.0
*
*  Description:
*    This file provides the power management source code to API for the
*    Counter.  
*
*   Note:
*     None
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "Counter.h"

static Counter_backupStruct Counter_backup;


/*******************************************************************************
* Function Name: Counter_SaveConfig
********************************************************************************
* Summary:
*     Save the current user configuration
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Counter_backup:  Variables of this global structure are modified to 
*  store the values of non retention configuration registers when Sleep() API is 
*  called.
*
*******************************************************************************/
void Counter_SaveConfig(void) 
{
    #if (!Counter_UsingFixedFunction)

        Counter_backup.CounterUdb = Counter_ReadCounter();

        #if(!Counter_ControlRegRemoved)
            Counter_backup.CounterControlRegister = Counter_ReadControlRegister();
        #endif /* (!Counter_ControlRegRemoved) */

    #endif /* (!Counter_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: Counter_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Counter_backup:  Variables of this global structure are used to 
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Counter_RestoreConfig(void) 
{      
    #if (!Counter_UsingFixedFunction)

       Counter_WriteCounter(Counter_backup.CounterUdb);

        #if(!Counter_ControlRegRemoved)
            Counter_WriteControlRegister(Counter_backup.CounterControlRegister);
        #endif /* (!Counter_ControlRegRemoved) */

    #endif /* (!Counter_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: Counter_Sleep
********************************************************************************
* Summary:
*     Stop and Save the user configuration
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Counter_backup.enableState:  Is modified depending on the enable 
*  state of the block before entering sleep mode.
*
*******************************************************************************/
void Counter_Sleep(void) 
{
    #if(!Counter_ControlRegRemoved)
        /* Save Counter's enable state */
        if(Counter_CTRL_ENABLE == (Counter_CONTROL & Counter_CTRL_ENABLE))
        {
            /* Counter is enabled */
            Counter_backup.CounterEnableState = 1u;
        }
        else
        {
            /* Counter is disabled */
            Counter_backup.CounterEnableState = 0u;
        }
    #else
        Counter_backup.CounterEnableState = 1u;
        if(Counter_backup.CounterEnableState != 0u)
        {
            Counter_backup.CounterEnableState = 0u;
        }
    #endif /* (!Counter_ControlRegRemoved) */
    
    Counter_Stop();
    Counter_SaveConfig();
}


/*******************************************************************************
* Function Name: Counter_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*  
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Counter_backup.enableState:  Is used to restore the enable state of 
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Counter_Wakeup(void) 
{
    Counter_RestoreConfig();
    #if(!Counter_ControlRegRemoved)
        if(Counter_backup.CounterEnableState == 1u)
        {
            /* Enable Counter's operation */
            Counter_Enable();
        } /* Do nothing if Counter was disabled before */    
    #endif /* (!Counter_ControlRegRemoved) */
    
}


/* [] END OF FILE */
