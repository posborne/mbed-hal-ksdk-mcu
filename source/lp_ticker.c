/* mbed Microcontroller Library
 * Copyright (c) 2015 ARM Limited
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
#include "cmsis.h"
#include "fsl_clock_manager.h"
#include "fsl_lptmr_hal.h"
#include "fsl_osc_hal.h"
#include "fsl_rtc_hal.h"
#include "device.h"

#if DEVICE_LOWPOWERTIMER

#include "lp_ticker_api.h"

/*
The Low power  timer for KSDK devices uses 2 timers. The timestamp is 32bit number,
however LPTMR contains only 16-bit timer. In addition, RTC contains only seconds alarm interrupt.
 
When adding a new event, if timestamp fits within LPTMR 16bit counter, its interrupt is set only,
otherwise RTC seconds alarm is used, and the leftover is for LPTMR timer.
*/

static int lp_ticker_inited = 0;
static int lp_timer_schedule = 0;
static void lptmr_isr(void);
static void rct_isr(void);

void lp_ticker_init(void) {
    if (lp_ticker_inited) {
        return;
    }
    lp_ticker_inited = 1;

    // RTC might be configured already, don't reset it
    if (!RTC_HAL_IsCounterEnabled(RTC_BASE)) {
        // select RTC for OSC32KSEL
        SIM->SOPT1 &= ~SIM_SOPT1_OSC32KSEL_MASK;
        SIM->SOPT1 |= SIM_SOPT1_OSC32KSEL(2);
        // configure RTC
        SIM_HAL_EnableRtcClock(SIM_BASE, 0U);
        RTC_HAL_Init(RTC_BASE);
        RTC_HAL_Enable(RTC_BASE);
        for (volatile uint32_t wait_count = 0; wait_count < 1000000; wait_count++);
        RTC_HAL_SetAlarmIntCmd(RTC_BASE, false);
        RTC_HAL_SetSecsIntCmd(RTC_BASE, false);
        RTC_HAL_SetAlarmReg(RTC_BASE, 0);
        RTC_HAL_EnableCounter(RTC_BASE, true);
    }
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_SetVector(RTC_IRQn, (uint32_t)rct_isr);
    NVIC_EnableIRQ(RTC_IRQn);

    // configure LPTMR
    CLOCK_SYS_EnableLptimerClock(0);
    LPTMR0_CSR = 0x00;
    LPTMR0_PSR = 0x00;
    LPTMR0_CMR = 0x00;
    LPTMR_HAL_SetTimerModeMode(LPTMR0_BASE, kLptmrTimerModeTimeCounter);
    LPTMR0_PSR |= LPTMR_PSR_PCS(0x2) | LPTMR_PSR_PBYP_MASK;
    LPTMR_HAL_SetFreeRunningCmd(LPTMR0_BASE, 0);
    IRQn_Type timer_irq[] = LPTMR_IRQS;
    NVIC_SetVector(timer_irq[0], (uint32_t)lptmr_isr);
    NVIC_EnableIRQ(timer_irq[0]);
}

// 4095 seconds is max, closest to uint32t max
#define RTC_TIMER_BITS (12)
#define RTC_TIMER_MASK ((1 << RTC_TIMER_BITS) - 1)

uint32_t lp_ticker_read() {
    if (!lp_ticker_inited) {
        lp_ticker_init();
    }
    uint32_t temp_pre = RTC_HAL_GetPrescaler(RTC_BASE);
    // RTC sec might be GT 4095, therfore use modulo the max allowance
    uint32_t temp_sec = RTC_HAL_GetSecsReg(RTC_BASE) % (1 << RTC_TIMER_BITS);
    while ((temp_pre != RTC_HAL_GetPrescaler(RTC_BASE)) || (temp_sec != (RTC_HAL_GetSecsReg(RTC_BASE) & RTC_TIMER_MASK))) {
        temp_pre = RTC_HAL_GetPrescaler(RTC_BASE);
        temp_sec = RTC_HAL_GetSecsReg(RTC_BASE) & RTC_TIMER_MASK;
    }
    uint32_t ret = ((temp_sec * 1000000) + (((uint64_t)temp_pre * 1000000) / 32768));
    return ret;
}

void lp_ticker_disable_interrupt(void) {
     LPTMR_HAL_SetIntCmd(LPTMR0_BASE, 0);
     RTC_HAL_SetAlarmIntCmd(RTC_BASE, false);
}

void lp_ticker_clear_interrupt(void) {
    LPTMR_HAL_ClearIntFlag(LPTMR0_BASE);
    RTC_HAL_SetAlarmReg(RTC_BASE, 0); //writing clears the flag
}

static void lptmr_isr(void)
{
    LPTMR_HAL_ClearIntFlag(LPTMR0_BASE);
    LPTMR_HAL_SetIntCmd(LPTMR0_BASE, 0);
    LPTMR_HAL_Disable(LPTMR0_BASE);
    // we are the last timeout, go up to report
    lp_ticker_irq_handler();
}

static void rct_isr(void)
{
    RTC_HAL_SetAlarmIntCmd(RTC_BASE, false);
    RTC_HAL_SetAlarmReg(RTC_BASE, 0);

    if (lp_timer_schedule) {
        // schedule LPTMR, restart counter and set compare
        LPTMR_HAL_Disable(LPTMR0_BASE);
        LPTMR_HAL_SetCompareValue(LPTMR0_BASE, lp_timer_schedule);
        LPTMR_HAL_Enable(LPTMR0_BASE);
        LPTMR_HAL_SetIntCmd(LPTMR0_BASE, 1);
        lp_timer_schedule = 0;
    } else {
        // RTC was only scheduled, go up to report
        lp_ticker_irq_handler();
    }
}

void lp_ticker_set_interrupt(timestamp_t timestamp) {
    uint32_t now = lp_ticker_read();
    uint32_t delta = timestamp > now ? timestamp - now : (uint32_t)((uint64_t)timestamp + 0xFFFFFFFF - now);
    uint32_t ticks = (delta / 30) + 1;
    lp_timer_schedule = 0;

    RTC_HAL_EnableCounter(RTC_BASE, false);
    if (ticks > 0xFFFE) {
        ticks -= 0x7FFF + 1 - RTC_HAL_GetPrescaler(RTC_BASE);
        uint32_t seconds = RTC_HAL_GetSecsReg(RTC_BASE);
        while (ticks > 0xFFFF) {
            ticks -= 0x7FFF + 1;
            seconds++;
        }
        RTC_HAL_SetAlarmReg(RTC_BASE, seconds);
        RTC_HAL_SetAlarmIntCmd(RTC_BASE, true);
        // the lp timer will be triggered once RTC alarm is set
        lp_timer_schedule = ticks;
    } else {
        // restart counter, set compare
        LPTMR_HAL_Disable(LPTMR0_BASE);
        LPTMR_HAL_SetCompareValue(LPTMR0_BASE, ticks);
        LPTMR_HAL_Enable(LPTMR0_BASE);
        LPTMR_HAL_SetIntCmd(LPTMR0_BASE, 1);
    }
    RTC_HAL_EnableCounter(RTC_BASE, true);
}

#endif
