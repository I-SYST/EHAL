/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "dfu_ble_svc.h"
#include <string.h>
#include "nrf_error.h"
#include "crc16.h"

static dfu_ble_peer_data_t m_peer_data;// __attribute__((section("NoInit"), zero_init));     /**< This variable should be placed in a non initialized RAM section in order to be valid upon soft reset from application into bootloader. */
static uint16_t            m_peer_data_crc;// __attribute__((section("NoInit"), zero_init)); /**< CRC variable to ensure the integrity of the peer data provided. */


/**@brief Function for setting the peer data from application in bootloader before reset.
 *
 * @param[in] p_peer_data  Pointer to the peer data containing keys for the connection.
 *
 * @retval NRF_SUCCES      The data was set succesfully.
 * @retval NRF_ERROR_NULL  If a null pointer was passed as argument.
 */
static uint32_t dfu_ble_set_peer_data(dfu_ble_peer_data_t * p_peer_data)
{
    if (p_peer_data == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t src = (uint32_t)p_peer_data;
    uint32_t dst = (uint32_t)&m_peer_data;
    uint32_t len = dst - src;

    if (src == dst)
    {
        // Do nothing as source and destination are identical, just calculate crc below.
    }
    else if (len < sizeof(dfu_ble_peer_data_t))
    {
        uint32_t i = 0;

        dst += sizeof(dfu_ble_peer_data_t);
        src += sizeof(dfu_ble_peer_data_t);

        // Copy byte wise backwards when facing overlapping structures.
        while (i++ <= sizeof(dfu_ble_peer_data_t))
        {
            *((uint8_t *)dst--) = *((uint8_t *)src--);
        }
    }
    else
    {
        memcpy((void *)dst, (void *)src, sizeof(dfu_ble_peer_data_t));
    }

    m_peer_data_crc = crc16_compute((uint8_t *)&m_peer_data, sizeof(m_peer_data), NULL);

    return NRF_SUCCESS;
}


/**@brief   Function for handling second stage of SuperVisor Calls (SVC).
 *
 * @details The function will use svc_num to call the corresponding SVC function.
 *
 * @param[in] svc_num    SVC number for function to be executed
 * @param[in] p_svc_args Argument list for the SVC.
 *
 * @return This function returns the error value of the SVC return. For further details, please
 *         refer to the details of the SVC implementation itself.
 *         @ref NRF_ERROR_SVC_HANDLER_MISSING is returned if no SVC handler is implemented for the
 *         provided svc_num.
 */
void C_SVC_Handler(uint8_t svc_num, uint32_t * p_svc_args)
{
    switch (svc_num)
    {
        case DFU_BLE_SVC_SET_PEER_DATA:
            p_svc_args[0] = dfu_ble_set_peer_data((dfu_ble_peer_data_t *)p_svc_args[0]);
            break;

        default:
            p_svc_args[0] = NRF_ERROR_SVC_HANDLER_MISSING;
            break;
    }

    return;
}


/**@brief   Function for handling the first stage of SuperVisor Calls (SVC) in assembly.
 *
 * @details The function will use the link register (LR) to determine the stack (PSP or MSP) to be
 *          used and then decode the SVC number afterwards. After decoding the SVC number then
 *          @ref C_SVC_Handler is called for further processing of the SVC.
 */
void SVC_Handler(void)
{
__asm ("EXC_RETURN_CMD_PSP  = 0xFFFFFFFD\n"	//  ; EXC_RETURN using PSP for ARM Cortex. If Link register contains this value it indicates the PSP was used before the SVC, otherwise the MSP was used.

    //".IMPORT C_SVC_Handler\n"
    "LDR   R0, =EXC_RETURN_CMD_PSP\n" // ; Load the EXC_RETURN into R0 to be able to compare against LR to determine stack pointer used.
    "CMP   R0, LR\n"                // ; Compare the link register with R0. If equal then PSP was used, otherwise MSP was used before SVC.
    "BNE   UseMSP\n"                // ; Branch to code fetching SVC arguments using MSP.
    "MRS   R1, PSP\n"               // ; Move PSP into R1.
    "B     Call_C_SVC_Handler\n"    // ; Branch to Call_C_SVC_Handler below.
"UseMSP:\n"
    "MRS   R1, MSP\n"               // ; MSP was used, therefore Move MSP into R1.
"Call_C_SVC_Handler:\n"
    "LDR   R0, [R1, #24]\n"         // ; The arguments for the SVC was stacked. R1 contains Stack Pointer, the values stacked before SVC are R0, R1, R2, R3, R12, LR, PC (Return address), xPSR.
                                    // ; R1 contains current SP so the PC of the stacked frame is at SP + 6 words (24 bytes). We load the PC into R0.
    "SUB  R0, R0, #2\n"             // ; The PC before the SVC is in R0. We subtract 2 to get the address prior to the instruction executed where the SVC number is located.
    "LDRB  R0, [R0]\n"              // ; SVC instruction low octet: Load the byte at the address before the PC to fetch the SVC number.
    "LDR   R2, =C_SVC_Handler\n"	// ; Load address of C implementation of SVC handler.
    "BX    R2\n"               		// ; Branch to C implementation of SVC handler. R0 is now the SVC number, R1 is the StackPointer where the arguments (R0-R3) of the original SVC are located.
    ".ALIGN\n");
}


uint32_t dfu_ble_get_peer_data(dfu_ble_peer_data_t * p_peer_data)
{
    uint16_t crc = crc16_compute((uint8_t *)&m_peer_data, sizeof(m_peer_data), NULL);

    if (crc != m_peer_data_crc)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    *p_peer_data = m_peer_data;

    // corrupt CRC to invalidate shared information.
    m_peer_data_crc++;

    return NRF_SUCCESS;
}

