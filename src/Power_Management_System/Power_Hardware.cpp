#include "Power_Internal.h"

LOG_MODULE_DECLARE(PWR_SYS, LOG_LEVEL_INF);

namespace pm_internal {

#ifndef CONFIG_BOARD_MPS2_AN386

static uint32_t s_usb_prev_gccfg = 0;
#define PM_USB_PCGCCTL   (*(volatile uint32_t*)((uint32_t)USB_OTG_FS + 0xE00U))
#define PM_USB_DEVICE    ((USB_OTG_DeviceTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_DEVICE_BASE))

void usb_disable_hardware_and_release_pins() {
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    __DSB();

    s_usb_prev_gccfg = USB_OTG_FS->GCCFG;
    PM_USB_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;    // soft-disconnect: DP pull-up off

    USB_OTG_FS->GCCFG = 0U;
    USB_OTG_FS->GOTGCTL = 0U;
    __DSB();

    PM_USB_PCGCCTL |= 0x3U;                      // STOPPCLK | GATEHCLK: stop PHY + HCLK inside the core

    volatile uint32_t dummy = USB_OTG_FS->GCCFG;
    (void)dummy;

    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    __DSB();
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    __DSB();

    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    __HAL_RCC_USB_OTG_FS_CLK_SLEEP_DISABLE();
}

void usb_restore_pins_and_reenable() {
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    __DSB();

    PM_USB_PCGCCTL &= ~0x3U;                     // ungate the core before touching it
    SET_BIT(USB_OTG_FS->GCCFG, s_usb_prev_gccfg | USB_OTG_GCCFG_PWRDWN);

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIOA->AFR[1]   = (GPIOA->AFR[1] & ~(0xFFU << 12)) | (0xAAU << 12);
    GPIOA->MODER    = (GPIOA->MODER & ~(0xFU << 22)) | (0xAU << 22);
    GPIOA->OSPEEDR |= (0xFU << 22);  // Very High Speed
    GPIOA->PUPDR   &= ~(0xFU << 22); // No Pull

    k_msleep(100);
}

#endif
#ifndef CONFIG_BOARD_MPS2_AN386
GPIO_TypeDef* const GPIO_PORTS[8] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH};

static inline uint32_t held_pin_pull(GPIO_TypeDef* gpio, int pin, uint32_t level, uint32_t pupdr_snap) {
    uint32_t pull = (pupdr_snap >> (pin * 2)) & 0x3U;   // 1 = pull-up, 2 = pull-down
    bool open_drain = ((gpio->OTYPER >> pin) & 0x1U) != 0U;

    if (open_drain && level) {
        return pull;
    }
    return 0U;
}

static uint32_t s_stop_prev_ahb2enr = 0;

uint16_t s_stop_od_af_mask[8]  = {};
uint16_t s_stop_od_low_mask[8] = {};
uint32_t s_stop_applied_moder[8] = {};
uint32_t s_stop_applied_pupdr[8] = {};

void gpio_save_and_set_all_analog(GpioSnapshot &snap, uint32_t &prev_ahb1enr) {
    prev_ahb1enr = RCC->AHB1ENR;
    s_stop_prev_ahb2enr = RCC->AHB2ENR;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __DSB();

    for (int port = 0; port < 8; ++port) {
        GPIO_TypeDef* gpio = GPIO_PORTS[port];

        snap.moder[port]   = gpio->MODER;
        snap.pupdr[port]   = gpio->PUPDR;
        snap.otyper[port]  = gpio->OTYPER;
        snap.ospeedr[port] = gpio->OSPEEDR;
        snap.odr[port]     = gpio->ODR;

        {
            uint32_t od_af = 0U;
            for (int pin = 0; pin < 16; ++pin) {
                if ((((snap.moder[port] >> (pin * 2)) & 0x3U) == 0x2U) &&
                    (((snap.otyper[port] >> pin) & 0x1U) != 0U)) {
                    od_af |= (1U << pin);
                }
            }
            s_stop_od_af_mask[port]  = static_cast<uint16_t>(od_af);
            s_stop_od_low_mask[port] = static_cast<uint16_t>(od_af & ~gpio->IDR);
        }

        uint32_t new_moder = 0xFFFFFFFFU;
        uint32_t new_pupdr = 0U;

        for (int pin = 0; pin < 16; ++pin) {
            if ((port == 0 && (pin == 13 || pin == 14 || pin == 15)) || (port == 1 && (pin == 3 || pin == 4))) {
                new_moder |= (0x3U << (pin * 2));
                new_pupdr &= ~(0x3U << (pin * 2));
                continue;
            }

            if (port == 2 && (pin == 14 || pin == 15)) {
                new_moder |= (0x3U << (pin * 2));
                new_pupdr &= ~(0x3U << (pin * 2));
                continue;
            }

            if (port == 0 && ((pin >= 9 && pin <= 12) || pin == 2 || pin == 3)) {
                new_moder |= (0x3U << (pin * 2));
                new_pupdr &= ~(0x3U << (pin * 2));
                continue;
            }

            uint32_t pin_mode = (snap.moder[port] >> (pin * 2)) & 0x3U;

            if (pin_mode == 0x01U) {
                uint32_t out_level = (gpio->ODR >> pin) & 0x1U;
                new_moder &= ~(0x3U << (pin * 2));
                new_moder |=  (0x1U << (pin * 2));
                new_pupdr |= (held_pin_pull(gpio, pin, out_level, snap.pupdr[port]) << (pin * 2));
            }
            else if (pin_mode == 0x02U) {
                uint32_t current_level = (gpio->IDR >> pin) & 0x1U;
                const bool open_drain = ((gpio->OTYPER >> pin) & 0x1U) != 0U;
                const bool drive_it   = (STOP_AF_DRIVE_PIN_MASK[port] & (1U << pin)) != 0U;

                if (open_drain || drive_it) {
                    if (current_level) {
                        gpio->ODR |= (1U << pin);
                    } else {
                        gpio->ODR &= ~(1U << pin);
                    }
                    new_moder &= ~(0x3U << (pin * 2));
                    new_moder |=  (0x1U << (pin * 2));
                    new_pupdr |= (held_pin_pull(gpio, pin, current_level, snap.pupdr[port]) << (pin * 2));
                } else {
                    new_moder &= ~(0x3U << (pin * 2));                         // 00 = input
                    new_pupdr |= ((current_level ? 0x1U : 0x2U) << (pin * 2)); // 01 up / 10 down
                }
            }
            else {
                new_moder |= (0x3U << (pin * 2));
            }
        }

        gpio->PUPDR   = new_pupdr;
        gpio->MODER   = new_moder;
        gpio->OSPEEDR = 0U; // Lowest slew rate in STOP
        s_stop_applied_moder[port] = new_moder;
        s_stop_applied_pupdr[port] = new_pupdr;
    }

    __DSB();

    RCC->AHB1ENR &= ~0x000000FFU;
}

void gpio_restore_after_wake(const GpioSnapshot &snap, uint32_t prev_ahb1enr) {
    RCC->AHB1ENR |= 0x000000FFU;
    __DSB();

    for (int i = 0; i < 8; ++i) {
        GPIO_PORTS[i]->ODR     = snap.odr[i];
        GPIO_PORTS[i]->OTYPER  = snap.otyper[i];
        GPIO_PORTS[i]->OSPEEDR = snap.ospeedr[i];
        GPIO_PORTS[i]->PUPDR   = snap.pupdr[i];
        GPIO_PORTS[i]->MODER   = snap.moder[i];
    }

    RCC->AHB1ENR = prev_ahb1enr;
    RCC->AHB2ENR = s_stop_prev_ahb2enr;
    __DSB();
}

void adc_save_and_disable(AdcSnapshot &snap) {
    __HAL_RCC_ADC1_CLK_ENABLE();
#ifdef __HAL_RCC_ADC2_CLK_ENABLE
    __HAL_RCC_ADC2_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC3_CLK_ENABLE
    __HAL_RCC_ADC3_CLK_ENABLE();
#endif
    __DSB();

    snap.cr1_1 = ADC1->CR1;
    snap.cr2_1 = ADC1->CR2;
    ADC1->CR2  = 0U;
    ADC1->CR1  = 0U;

#ifdef ADC2
    snap.cr1_2 = ADC2->CR1;
    snap.cr2_2 = ADC2->CR2;
    ADC2->CR2  = 0U;
    ADC2->CR1  = 0U;
#endif

#ifdef ADC3
    snap.cr1_3 = ADC3->CR1;
    snap.cr2_3 = ADC3->CR2;
    ADC3->CR2  = 0U;
    ADC3->CR1  = 0U;
#endif

    snap.ccr = ADC123_COMMON->CCR;
    // VREFINT/temp sensor buffer + VBAT bridge + ADC prescaler completely off
    ADC123_COMMON->CCR = 0U;

    volatile uint32_t dummy = ADC123_COMMON->CCR;
    (void)dummy;

    __HAL_RCC_ADC1_CLK_DISABLE();
#ifdef __HAL_RCC_ADC2_CLK_DISABLE
    __HAL_RCC_ADC2_CLK_DISABLE();
#endif
#ifdef __HAL_RCC_ADC3_CLK_DISABLE
    __HAL_RCC_ADC3_CLK_DISABLE();
#endif
}

void adc_restore_after_wake(const AdcSnapshot &snap) {
    __HAL_RCC_ADC1_CLK_ENABLE();
#ifdef __HAL_RCC_ADC2_CLK_ENABLE
    __HAL_RCC_ADC2_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_ADC3_CLK_ENABLE
    __HAL_RCC_ADC3_CLK_ENABLE();
#endif
    __DSB();

    ADC123_COMMON->CCR = snap.ccr;
    ADC1->CR1 = snap.cr1_1;
    ADC1->CR2 = snap.cr2_1;
#ifdef ADC2
    ADC2->CR1 = snap.cr1_2;
    ADC2->CR2 = snap.cr2_2;
#endif
#ifdef ADC3
    ADC3->CR1 = snap.cr1_3;
    ADC3->CR2 = snap.cr2_3;
#endif
    __DSB();
}

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

void dac_save_and_disable(uint32_t &prev_cr) {
#ifdef DAC
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_DACEN);
    __DSB();
    prev_cr = DAC->CR;
    DAC->CR = 0U;   // output buffers & channels completely off (they ignore clock gating)
    volatile uint32_t dummy = DAC->CR;
    (void)dummy;
    CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_DACEN);
#else
    prev_cr = 0U;
#endif
}

void dac_restore_after_wake(uint32_t prev_cr) {
#ifdef DAC
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_DACEN);
    __DSB();
    DAC->CR = prev_cr;
#else
    (void)prev_cr;
#endif
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
#endif

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

void suspend_background_threads() {
    k_thread_suspend(processor_tid);
    k_thread_suspend(producer_tid);
    k_thread_suspend(logger_tid);
    k_thread_suspend(battery_tid);
    k_thread_suspend(shell_tid);
    k_thread_suspend(bms_comm_tid);
    k_thread_suspend(hr_prod_tid);
    k_thread_suspend(disp_cons_tid);
    k_thread_suspend(mem_mon_tid);
    k_thread_suspend(trace_tid);
}

void resume_background_threads() {
    k_thread_resume(processor_tid);
    k_thread_resume(producer_tid);
    k_thread_resume(logger_tid);
    k_thread_resume(battery_tid);
    k_thread_resume(shell_tid);
    k_thread_resume(bms_comm_tid);
    k_thread_resume(hr_prod_tid);
    k_thread_resume(disp_cons_tid);
    k_thread_resume(mem_mon_tid);
    k_thread_resume(trace_tid);
}

} // namespace pm_internal

using namespace pm_internal;

namespace {
void usb_reconnect_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

#ifndef CONFIG_BOARD_MPS2_AN386
    usb_restore_pins_and_reenable();
#ifdef CONFIG_USB_DEVICE_STACK
    int err = usb_dc_attach();
    if (err == 0) {
        LOG_INF("USB re-attached after STOP wake; awaiting host re-enumeration");
    } else {
        LOG_ERR("Failed to re-attach USB after STOP wake (err %d)", err);
    }
#endif
#endif
}
}

K_WORK_DELAYABLE_DEFINE(usb_reconnect_work, usb_reconnect_work_handler);

#ifdef CONFIG_BOARD_MPS2_AN386

bool pm_device_suspend_tolerant(const struct device* dev, const char* name) {
    if (!dev) {
        return true;
    }

    int rc = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
    if (rc == 0 || rc == -EALREADY || rc == -ENOSYS || rc == -ENOTSUP) {
        return true;
    }

    LOG_ERR("%s suspend failed: %d", name, rc);
    return false;
}

bool pm_device_resume_tolerant(const struct device* dev, const char* name) {
    if (!dev) {
        return true;
    }

    int rc = pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
    if (rc == 0 || rc == -EALREADY || rc == -ENOSYS || rc == -ENOTSUP) {
        return true;
    }

    LOG_ERR("%s resume failed: %d", name, rc);
    return false;
}
#endif
