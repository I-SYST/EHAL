/*--------------------------------------------------------------------------
File   : system_nrf51.c

Author : Hoang Nguyen Hoan          Aug. 16, 2014

Desc   : Generic system code for ARM CMSIS with GCC compiler

Copyright (c) 2014, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <stdint.h>

#include "nrf.h"

// nRF51 support two crystal frequency
#define XTAL_FREQ_16MHZ       16000000UL
#define XTAL_FREQ_32MHZ       32000000UL


#define SYSTEM_CLOCK

uint32_t SystemCoreClock = 16000000UL;

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 16000000UL;
}

void SystemInit(void)
{
	NRF_CLOCK->XTALFREQ = CLOCK_XTALFREQ_XTALFREQ_16MHz;

	/* If desired, switch off the unused RAM to lower consumption by the use of RAMON register.
       It can also be done in the application main() function. */

    /* Prepare the peripherals for use as indicated by the PAN 26 "System: Manual setup is required
       to enable the use of peripherals" found at Product Anomaly document for your device found at
       https://www.nordicsemi.com/. The side effect of executing these instructions in the devices 
       that do not need it is that the new peripherals in the second generation devices (LPCOMP for
       example) will not be available. */
    if (is_manual_peripheral_setup_needed())
    {
        *(uint32_t volatile *)0x40000504 = 0xC007FFDF;
        *(uint32_t volatile *)0x40006C18 = 0x00008000;
    }
    
    /* Disable PROTENSET registers under debug, as indicated by PAN 59 "MPU: Reset value of DISABLEINDEBUG
       register is incorrect" found at Product Anomaly document four your device found at 
       https://www.nordicsemi.com/. There is no side effect of using these instruction if not needed. */
    if (is_disabled_in_debug_needed())
    {
        NRF_MPU->DISABLEINDEBUG = MPU_DISABLEINDEBUG_DISABLEINDEBUG_Disabled << MPU_DISABLEINDEBUG_DISABLEINDEBUG_Pos;
    }
}


static bool is_manual_peripheral_setup_needed(void) 
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x1) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
    {
        if ((((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x00) && (((*(uint32_t *)0xF0000FEC) & 0x000000F0) == 0x0))
        {
            return true;
        }
        if ((((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x10) && (((*(uint32_t *)0xF0000FEC) & 0x000000F0) == 0x0))
        {
            return true;
        }
        if ((((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30) && (((*(uint32_t *)0xF0000FEC) & 0x000000F0) == 0x0))
        {
            return true;
        }
    }
    
    return false;
}

static bool is_disabled_in_debug_needed(void) 
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x1) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
    {
        if ((((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x40) && (((*(uint32_t *)0xF0000FEC) & 0x000000F0) == 0x0))
        {
            return true;
        }
    }
    
    return false;
}

/*lint --flb "Leave library region" */
