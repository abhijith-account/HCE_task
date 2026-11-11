// Power_Hardware_StopPwr.cpp
// STOP-mode power-controller handling for the STM32F4: PWR / SYSCFG / DBGMCU configuration before STOP, re-asserting the STOP bits
// right before each WFI, executing the WFI itself (PRIMASK=1, SLEEPDEEP), and restoring the power registers after wake.
// Real hardware only. Declarations: Power_Internal.h.

#include "Power_Internal.h"

namespace pm_internal {

#ifndef CONFIG_BOARD_MPS2_AN386

static uint32_t s_prev_pwr_csr = 0;
static uint32_t s_prev_syscfg_cmpcr = 0;

uint32_t s_stop_dbgmcu_after_write = 0;   // DBGMCU_CR read back right after stop_dbg_disable() wrote it

static constexpr uint32_t STOP_REG_BITS = PWR_CR_PDDS | PWR_CR_LPDS | PWR_CR_FPDS
#ifdef PWR_CR_LPUDS
    | PWR_CR_LPUDS
#endif
#ifdef PWR_CR_LPLVDS
    | PWR_CR_LPLVDS
#endif
#ifdef PWR_CR_MRLVDS
    | PWR_CR_MRLVDS
#endif
#ifdef PWR_CR_MRUDS
    | PWR_CR_MRUDS
#endif
#ifdef PWR_CR_UDEN
    | PWR_CR_UDEN
#endif
    ;

static void apply_stop_pwr_bits() {
    stop_dbg_disable();

    MODIFY_REG(PWR->CR, PWR_CR_VOS, PWR_CR_VOS_0);
    uint32_t clear_mask = STOP_REG_BITS | PWR_CR_PVDE;
#ifdef PWR_CR_ODEN
    clear_mask |= PWR_CR_ODEN | PWR_CR_ODSWEN;
#endif
    CLEAR_BIT(PWR->CR, clear_mask);
    SET_BIT(PWR->CR, PWR_CR_CWUF | PWR_CR_CSBF);
    __DSB();
}

void stop_configure_power(uint32_t &prev_pwr_cr) {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __DSB();

    prev_pwr_cr = PWR->CR;
    s_prev_pwr_csr = PWR->CSR;
    s_prev_syscfg_cmpcr = SYSCFG->CMPCR;

    CLEAR_BIT(SYSCFG->CMPCR, SYSCFG_CMPCR_CMP_PD);

    SET_BIT(PWR->CR, PWR_CR_DBP);
    __DSB();
    CLEAR_BIT(PWR->CSR, PWR_CSR_BRE);

    apply_stop_pwr_bits();
}

void stop_reassert_power() {
    __HAL_RCC_PWR_CLK_ENABLE();
    __DSB();
    apply_stop_pwr_bits();
}

void stop_dbg_disable() {
    HAL_DBGMCU_DisableDBGSleepMode();
    HAL_DBGMCU_DisableDBGStopMode();
    HAL_DBGMCU_DisableDBGStandbyMode();

    for (int attempt = 0; attempt < 8; ++attempt) {
        DBGMCU->CR = 0U;
        __DSB();
        if (DBGMCU->CR == 0U) {
            break;
        }
    }
    DWT->CTRL        = 0U;
    CoreDebug->DEMCR = 0U;
    __DSB();
    s_stop_dbgmcu_after_write = DBGMCU->CR;
}

HAL_StatusTypeDef stop_enter_wfi(uint32_t &out_pwr_csr_on_wake, uint32_t &out_icsr_on_wake) {
    __HAL_RCC_PWR_CLK_ENABLE();
    __DSB();

    stop_dbg_disable();
    CLEAR_BIT(PWR->CR, STOP_REG_BITS);   // main regulator, normal mode, flash on in Stop
    SET_BIT(PWR->CR, PWR_CR_CWUF | PWR_CR_CSBF);   // clear wake-up / standby flags
    __DSB();

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
    __DSB();
    __ISB();

    stop_dbg_disable();
    __ISB();
    __WFI();
    __ISB();

    out_pwr_csr_on_wake = PWR->CSR;
    out_icsr_on_wake    = SCB->ICSR;

    CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
    __DSB();

    if (!primask) {
        __enable_irq();
    }

    return HAL_OK;
}

void stop_restore_power(uint32_t prev_pwr_cr) {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __DSB();

    if (s_prev_pwr_csr & PWR_CSR_BRE) {
        SET_BIT(PWR->CR, PWR_CR_DBP);
        __DSB();
        SET_BIT(PWR->CSR, PWR_CSR_BRE);
    }

    SYSCFG->CMPCR = s_prev_syscfg_cmpcr;

    uint32_t mask = PWR_CR_PDDS | PWR_CR_LPDS | PWR_CR_FPDS | PWR_CR_PVDE | PWR_CR_VOS;
#ifdef PWR_CR_LPUDS
    mask |= PWR_CR_LPUDS;
#endif
#ifdef PWR_CR_MRUDS
    mask |= PWR_CR_MRUDS;
#endif
#ifdef PWR_CR_MRLVDS
    mask |= PWR_CR_MRLVDS;
#endif
#ifdef PWR_CR_LPLVDS
    mask |= PWR_CR_LPLVDS;
#endif
#ifdef PWR_CR_UDEN
    mask |= PWR_CR_UDEN;
#endif
    PWR->CR = (PWR->CR & ~mask) | (prev_pwr_cr & mask);
    __DSB();
}

#endif // !CONFIG_BOARD_MPS2_AN386

} // namespace pm_internal
