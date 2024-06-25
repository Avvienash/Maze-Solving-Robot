/*******************************************************************************
* File Name: dir.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_dir_ALIASES_H) /* Pins dir_ALIASES_H */
#define CY_PINS_dir_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"


/***************************************
*              Constants        
***************************************/
#define dir_0			(dir__0__PC)
#define dir_0_INTR	((uint16)((uint16)0x0001u << dir__0__SHIFT))

#define dir_INTR_ALL	 ((uint16)(dir_0_INTR))

#endif /* End Pins dir_ALIASES_H */


/* [] END OF FILE */
