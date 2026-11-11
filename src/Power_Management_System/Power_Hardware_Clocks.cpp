// Power_Hardware_Clocks.cpp
// Clock handling for Idle and STOP: peripheral clock gating (APB1/APB2 ENR + LPENR) and the Idle clock downscale
// (HSI, HCLK divider, PLL / HSE / PLLI2S / PLLSAI off, voltage scale 3, Over-drive off) with its restore.
// The functions are compiled on both builds; their register access is guarded by CONFIG_BOARD_MPS2_AN386 inside each function.
// Declarations: Power_Internal.h.

#include "Power_Internal.h"

namespace pm_internal {

[[maybe_unused]] void periph_clocks_save_and_gate(PeripheralClockSnapshot &snap, uint32_t apb1_keep_mask, uint32_t apb2_keep_mask) {
#ifndef CONFIG_BOARD_MPS2_AN386
    snap.apb1enr   = RCC->APB1ENR;
    snap.apb2enr   = RCC->APB2ENR;
    snap.apb1lpenr = RCC->APB1LPENR;
    snap.apb2lpenr = RCC->APB2LPENR;

    RCC->APB1ENR   &= apb1_keep_mask;
    RCC->APB2ENR   &= apb2_keep_mask;
    RCC->APB1LPENR &= apb1_keep_mask;
    RCC->APB2LPENR &= apb2_keep_mask;
    __DSB();
#endif
}

[[maybe_unused]] void periph_clocks_restore(const PeripheralClockSnapshot &snap) {
#ifndef CONFIG_BOARD_MPS2_AN386
    RCC->APB1LPENR = snap.apb1lpenr;
    RCC->APB2LPENR = snap.apb2lpenr;
    RCC->APB1ENR   = snap.apb1enr;
    RCC->APB2ENR   = snap.apb2enr;
    __DSB();
#endif
}

void clock_save_state(ClockSnapshot &snap) {
#ifndef CONFIG_BOARD_MPS2_AN386
    __HAL_RCC_PWR_CLK_ENABLE();
    __DSB();
    snap.cr           = RCC->CR;
    snap.cfgr         = RCC->CFGR;
    snap.systick_ctrl = SysTick->CTRL;
    snap.systick_load = SysTick->LOAD;
    snap.flash_acr    = FLASH->ACR;
    snap.pwr_vos      = PWR->CR & PWR_CR_VOS;
#ifdef PWR_CR_ODEN
    snap.pwr_od       = PWR->CR & (PWR_CR_ODEN | PWR_CR_ODSWEN);
#else
    snap.pwr_od       = 0U;
#endif
#endif
}

void clock_downscale_for_idle(ClockSnapshot &snap, [[maybe_unused]] uint32_t hclk_div) {
    clock_save_state(snap);

#ifndef CONFIG_BOARD_MPS2_AN386
    const uint32_t hpre = (hclk_div == 1U) ? RCC_CFGR_HPRE_DIV1 : (hclk_div == 2U) ? RCC_CFGR_HPRE_DIV2 : RCC_CFGR_HPRE_DIV4;
    [[maybe_unused]] const uint32_t hclk_hz = 16000000U / ((hclk_div == 1U || hclk_div == 2U) ? hclk_div : 4U);
    __HAL_RCC_HSI_ENABLE();
    uint32_t timeout = 100000;
    while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) && --timeout) {}

    MODIFY_REG(RCC->CFGR,RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2,hpre | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1);

    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
    timeout = 100000;
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSI && --timeout) {}

#ifdef CONFIG_SYS_CLOCK_TICKS_PER_SEC
    uint32_t new_load = (hclk_hz / CONFIG_SYS_CLOCK_TICKS_PER_SEC) - 1U;
    if (new_load > 0U) {
        SysTick->LOAD = new_load;
        SysTick->VAL  = 0U;
    }
#endif

    FLASH->ACR &= ~(FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

    __HAL_RCC_PLL_DISABLE();
    {
        uint32_t pll_off_timeout = 100000;
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) && --pll_off_timeout) {}
    }
#ifdef PWR_CR_ODEN

    if ((PWR->CR & (PWR_CR_ODEN | PWR_CR_ODSWEN)) != 0U) {
        CLEAR_BIT(PWR->CR, PWR_CR_ODEN | PWR_CR_ODSWEN);
        uint32_t od_off_timeout = 100000;
        while ((PWR->CSR & (PWR_CSR_ODRDY | PWR_CSR_ODSWRDY)) && --od_off_timeout) {}
    }
#endif
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
#ifdef RCC_CR_PLLI2SON
    CLEAR_BIT(RCC->CR, RCC_CR_PLLI2SON);
#endif
#ifdef RCC_CR_PLLSAION
    CLEAR_BIT(RCC->CR, RCC_CR_PLLSAION);
#endif

    // Explicitly turn off HSE, CSS, and HSEBYP so PH0/PH1 return to GPIO control
    CLEAR_BIT(RCC->CR, RCC_CR_CSSON);
    CLEAR_BIT(RCC->CR, RCC_CR_HSEON);
    uint32_t hse_off_timeout = 1000000;
    while ((RCC->CR & RCC_CR_HSERDY) && --hse_off_timeout) { }
    CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
#endif
}

bool clock_restore_from_idle(const ClockSnapshot &snap)
{
#ifndef CONFIG_BOARD_MPS2_AN386
    __HAL_RCC_PWR_CLK_ENABLE();
    __DSB();

    MODIFY_REG(PWR->CR, PWR_CR_VOS, snap.pwr_vos);
    MODIFY_REG(RCC->CFGR,RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2, snap.cfgr & (RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2));
    FLASH->ACR = snap.flash_acr;

    if (snap.cr & RCC_CR_HSEBYP) {
        SET_BIT(RCC->CR, RCC_CR_HSEBYP);
    }

    if (snap.cr & RCC_CR_HSEON) {
        SET_BIT(RCC->CR, RCC_CR_HSEON);

        uint32_t timeout = 1000000;
        while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) && --timeout) {}

        if (timeout == 0) {
            return false;
        }
    }

    if (snap.cr & RCC_CR_PLLON) {
        __HAL_RCC_PLL_ENABLE();

        uint32_t timeout = 1000000;
        while (!__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) && --timeout) {}

        if (timeout == 0) {
            return false;
        }

        uint32_t vos_timeout = 1000000;
        while (!(PWR->CSR & PWR_CSR_VOSRDY) && --vos_timeout) {}

        if (vos_timeout == 0) {
            return false;
        }

#ifdef PWR_CR_ODEN

        if ((snap.pwr_od & PWR_CR_ODEN) != 0U) {
            SET_BIT(PWR->CR, PWR_CR_ODEN);
            uint32_t od_timeout = 1000000;
            while (!(PWR->CSR & PWR_CSR_ODRDY) && --od_timeout) {}
            if (od_timeout == 0) {
                return false;
            }
            if ((snap.pwr_od & PWR_CR_ODSWEN) != 0U) {
                SET_BIT(PWR->CR, PWR_CR_ODSWEN);
                od_timeout = 1000000;
                while (!(PWR->CSR & PWR_CSR_ODSWRDY) && --od_timeout) {}
                if (od_timeout == 0) {
                    return false;
                }
            }
        }
#endif
    }

#ifdef RCC_CR_PLLI2SON
    if (snap.cr & RCC_CR_PLLI2SON) {
        SET_BIT(RCC->CR, RCC_CR_PLLI2SON);
    }
#endif
#ifdef RCC_CR_PLLSAION
    if (snap.cr & RCC_CR_PLLSAION) {
        SET_BIT(RCC->CR, RCC_CR_PLLSAION);
    }
#endif

    uint32_t sw = snap.cfgr & RCC_CFGR_SW;
    __HAL_RCC_SYSCLK_CONFIG(sw);
    uint32_t expected_sws = sw << 2;

    uint32_t timeout2 = 1000000;
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != expected_sws && --timeout2) {}

    if (timeout2 == 0) {
        return false;
    }

    if (snap.systick_load > 0U) {
        SysTick->LOAD = snap.systick_load;
        SysTick->VAL  = 0U;
    }

    return true;
#else
    return true;
#endif
}

} // namespace pm_internal
