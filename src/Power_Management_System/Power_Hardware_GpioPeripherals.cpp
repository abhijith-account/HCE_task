// Power_Hardware_GpioPeripherals.cpp
// STOP-mode pin and peripheral handling for the STM32F4: USB OTG pin release / restore, GPIO save + analog-park + restore,
// ADC and DAC shut-down (they keep drawing current in STOP otherwise). Real hardware only; the MPS2 (host/QEMU) build has none of this.
// Declarations: Power_Internal.h. Clocks are in Power_Hardware_Clocks.cpp, PWR/WFI in Power_Hardware_StopPwr.cpp.

#include "Power_Internal.h"

namespace pm_internal {

#ifndef CONFIG_BOARD_MPS2_AN386

// ---- USB OTG FS: soft-disconnect, release PA11/PA12 for STOP, and bring them back ----
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

// ---- GPIO: snapshot every port, park all pins in analog/held states, restore after wake ----
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

// ---- ADC: save + power down ADC1/2/3 and the common block ----
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

// ---- DAC: output buffers keep drawing current in STOP ----
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

#endif // !CONFIG_BOARD_MPS2_AN386

} // namespace pm_internal
