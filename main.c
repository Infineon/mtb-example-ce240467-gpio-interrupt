/*******************************************************************************
* File Name:   main.c
*
* Description: This code example demonstrates the use of GPIO configured as an
*              input pin to generate interrupts.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define GPIO_INTERRUPT_PRIORITY 7
#define DELAY_SHORT_MS          (250)   /* milliseconds */
#define DELAY_LONG_MS           (500)   /* milliseconds */
#define LED_BLINK_COUNT         (4)

/*******************************************************************************
* Global Variables
*******************************************************************************/
volatile bool gpio_intr_flag = false;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
/* This structure initializes the Port5 interrupt for the NVIC */
cy_stc_sysint_t intrCfgBTN =
{
    .intrSrc = ioss_interrupts_sec_gpio_5_IRQn, /* Interrupt source is GPIO port 5 interrupt */
    .intrPriority = GPIO_INTERRUPT_PRIORITY /* Interrupt priority is 7 */
};

/*******************************************************************************
* Function Definitions
*******************************************************************************/
static void BTN_interrupt_handler_PDL();

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. This function configures and initializes the
* GPIO interrupt, update the delay on every GPIO interrupt, blinks the LED and
* enter in deepsleep mode.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t count = 0;
    uint32_t delay_led_blink = DELAY_LONG_MS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the user button */
    Cy_GPIO_Pin_SecFastInit(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN, CY_GPIO_DM_PULLUP, 1UL, HSIOM_SEL_GPIO);

    /* Configure GPIO5.0 interrupt vector */
    Cy_SysInt_Init(&intrCfgBTN, BTN_interrupt_handler_PDL);
    NVIC_EnableIRQ(ioss_interrupts_sec_gpio_5_IRQn);

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* Check the interrupt status */
        if (true == gpio_intr_flag)
        {
            gpio_intr_flag = false;

            /* Update LED toggle delay */
            if (DELAY_LONG_MS == delay_led_blink)
            {
                delay_led_blink = DELAY_SHORT_MS;
            }
            else
            {
                delay_led_blink = DELAY_LONG_MS;
            }
        }
        /* Blink LED four times */
        for (count = 0; count < LED_BLINK_COUNT; count++)
        {
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_ON); /* LED ON */
            Cy_SysLib_Delay(delay_led_blink);
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF); /* LED ON */
            Cy_SysLib_Delay(delay_led_blink);

        }
        /* Enter deep sleep mode */
        Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

    }
}

/*******************************************************************************
* Function Name: GPIO_Interrupt_handler_PDL
********************************************************************************
*
*  Summary:
*  GPIO interrupt handler for the PDL example.
*
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
static void BTN_interrupt_handler_PDL()
{
    /* Clear pin interrupt logic. Required to detect next interrupt */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
    gpio_intr_flag = true;

}
/* [] END OF FILE */
