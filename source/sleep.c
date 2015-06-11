/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sleep_api.h"
#include "cmsis.h"
#include "fsl_mcg_hal.h"
#include "fsl_smc_hal.h"

void sleep(void) {
    smc_power_mode_protection_config_t sleep_config = {
        .vlpProt = true,            /*!< VLP protect*/
        .llsProt = true,            /*!< LLS protect */
        .vllsProt = true,           /*!< VLLS protect*/
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
        .hsrunProt = true,          /*!< HSRUN protect */
#endif
    };
    SMC_HAL_SetProtection(SMC_BASE, &sleep_config);

    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
}

void deepsleep(void) {
    /* NOTE: this should me mcg_clock_select_t mcg_clock = CLOCK_HAL_GetClkSrcMode(MCG_BASE);
       However, at least with arm-none-eabi-gcc 4.8.3 20131129, the above call ends up doing a 32 bit access to the MCG
       registers, which results in a hard fault, since the MCG registers only accept 8 bit accesses apparently.
       Newer version of KSDK changed the way they access the MCG, so that should be fixed, but until we update to
       a newer KSDK, the code below should provide a compiler-agnostic fix
    */
    mcg_clock_select_t mcg_clock = (mcg_clock_select_t)(((*(volatile uint8_t*) HW_MCG_C1_ADDR(MCG_BASE)) & BM_MCG_C1_CLKS) >> BP_MCG_C1_CLKS);

    smc_power_mode_protection_config_t sleep_config = {
        .vlpProt = true,            /*!< VLP protect*/
        .llsProt = true,            /*!< LLS protect */
        .vllsProt = true,           /*!< VLLS protect*/
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
        .hsrunProt = true,          /*!< HSRUN protect */
#endif
    };
    SMC_HAL_SetProtection(SMC_BASE, &sleep_config);
    SMC->PMCTRL = SMC_PMCTRL_STOPM(2);

    //Deep sleep for ARM core:
    SCB->SCR = 1 << SCB_SCR_SLEEPDEEP_Pos;

    __WFI();

    //Switch back to PLL as clock source if needed
    //The interrupt that woke up the device will run at reduced speed
    if (mcg_clock == kMcgClkSelOut) {
        if (CLOCK_HAL_GetPllStatMode(MCG_BASE) == kMcgPllStatPllClkSel) {
            while (CLOCK_HAL_GetLock0Mode(MCG_BASE) == kMcgLockUnlocked);
        }
        CLOCK_HAL_SetClkSrcMode(MCG_BASE, kMcgClkSelOut);
    }
}
