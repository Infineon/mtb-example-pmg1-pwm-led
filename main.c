/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PMG1 PWM LED Example
*                    for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
* Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"


/*******************************************************************************
* Macros
*******************************************************************************/

/* Set the PWM duty cycle in percentage. */
#define PWM_DUTY_CYCLE     (10u)

/* Set the PWM period in count value in the range: 0 - 65535. Maximum value 65535 corresponds to 1 full round of count. */
#define PWM_PERIOD         (uint16_t)(1000)

/* PWM Compare value is calculated based on the required duty cycle */
#define PWM_COMPARE        (uint16_t)(PWM_PERIOD * PWM_DUTY_CYCLE / 100)

/*******************************************************************************
* Function Name: main
*******************************************************************************
* Summary:
* The main function performs the following actions:
*    1. Initializes the BSP
*    2. Initializes the TCPWM block as PWM
*    3. Sets the Period and Compare value for PWM
*
* Parameters:
*  void
*
* Return:
*  int
*
******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize PWM using the configuration structure generated using device configurator */
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(CYBSP_PWM_HW, CYBSP_PWM_NUM, &CYBSP_PWM_config))
    {
        /* Error handling */
        CY_ASSERT(0);
    }

    /* Set PWM Period count value*/
    /* Calling this API function overwrites the default period value stored in the TCPWM configuration structure */
    Cy_TCPWM_PWM_SetPeriod0(CYBSP_PWM_HW, CYBSP_PWM_NUM, PWM_PERIOD);

    /* Set PWM Compare value*/
    /* The Compare value is used to adjust the PWM duty cycle */
    /* Calling this API function overwrites the default compare value stored in the TCPWM configuration structure */
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_HW, CYBSP_PWM_NUM, PWM_COMPARE);

    /* Enable the TCPWM as PWM */
    Cy_TCPWM_PWM_Enable(CYBSP_PWM_HW, CYBSP_PWM_NUM);

    /* Start the PWM */
    Cy_TCPWM_TriggerReloadOrIndex(CYBSP_PWM_HW, CYBSP_PWM_MASK);

    for (;;)
    {

    }
}
/* [] END OF FILE */
