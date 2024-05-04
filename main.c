/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PMG1 PWM LED Example
*                    for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2022-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#include <stdio.h>
#include <inttypes.h>

/*******************************************************************************
* Macros
*******************************************************************************/

/* Set the PWM duty cycle in percentage. */
#define PWM_DUTY_CYCLE     (10u)

/* Set the PWM period in count value in the range: 0 - 65535. Maximum value 65535 corresponds to 1 full round of count. */
#define PWM_PERIOD         (uint16_t)(1000)

/* PWM Compare value is calculated based on the required duty cycle */
#define PWM_COMPARE        (uint16_t)(PWM_PERIOD * PWM_DUTY_CYCLE / 100)

/* CY ASSERT failure */
#define CY_ASSERT_FAILED   (0u)

/* Debug print macro to enable UART print */
#define DEBUG_PRINT        (0u)

#if DEBUG_PRINT
/* Structure for UART context */
cy_stc_scb_uart_context_t CYBSP_UART_context;
/* Variable used for tracking the print status */
volatile bool ENTER_LOOP = true;

/*******************************************************************************
* Function Name: check_status
********************************************************************************
* Summary:
*  Prints the error message.
*
* Parameters:
*  error_msg - message to print if any error encountered.
*  status - status obtained after evaluation.
*
* Return:
*  void
*
*******************************************************************************/
void check_status(char *message, cy_rslt_t status)
{
    char error_msg[50];

    sprintf(error_msg, "Error Code: 0x%08" PRIX32 "\n", status);

    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\nFAIL: ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, message);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, error_msg);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
}
#endif

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
    cy_en_tcpwm_status_t pwm_result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

#if DEBUG_PRINT

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Sequence to clear screen */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");

    /* Print "PWM LED" */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "****************** ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "PMG1 MCU: PWM LED");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "****************** \r\n\n");
#endif

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize PWM using the configuration structure generated using device configurator */
    pwm_result = Cy_TCPWM_PWM_Init(CYBSP_PWM_HW, CYBSP_PWM_NUM, &CYBSP_PWM_config);
    if (pwm_result != CY_TCPWM_SUCCESS)
    {
#if DEBUG_PRINT
        check_status("API Cy_TCPWM_Init failed with error code",pwm_result);
#endif
        CY_ASSERT(CY_ASSERT_FAILED);
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
#if DEBUG_PRINT
        if (ENTER_LOOP)
        {
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Entered for loop\r\n");
            ENTER_LOOP = false;
        }
#endif
    }
}
/* [] END OF FILE */
