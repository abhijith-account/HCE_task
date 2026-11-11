// Usb_Cdc_Facade.cpp
// UsbCdcFacade: the USB CDC-ACM transport underneath the shell. UART RX interrupt handler and ring buffer (overflow handling,
// backspace echo), DTR / cable detection, chunked transmit, and readLine() (CR/LF handling, over-long command discard).
// Layering: Usb_Shell_Runtime.cpp -> Usb_Shell_Commands.cpp -> this file. This file knows nothing about commands or threads.

#include "USB_CDC_Virtual_COM_Shell_Interface.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/__assert.h>
#include "Persistent_Configuration_System.h"
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <charconv>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <utility>
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h" 

LOG_MODULE_REGISTER(USB_CLI, LOG_LEVEL_INF);

bool g_usb_connected = false;

UsbCdcFacade::UsbCdcFacade() noexcept {
    dev = DEVICE_DT_GET(DT_ALIAS(cdc_acm_uart0));
    __ASSERT(dev != nullptr, "CDC ACM device missing from devicetree");
}

bool UsbCdcFacade::init() {
    if (initialized) {
        return true;
    }
    if (!device_is_ready(dev)) {
        LOG_ERR("Failed to enable USB device");
        return false;
    }
    const int ret = uart_irq_callback_user_data_set(dev, uartInterruptHandler, this);
    if (ret != 0) {
        LOG_ERR("Failed to set UART IRQ callback (err %d)", ret);
        return false;
    }
    
    initialized = true;
    LOG_INF("USB CDC Facade Initialized");
    return true;
}

bool UsbCdcFacade::isConnected() {
    if (!g_usb_connected) {
        if (dtr_ready) {
            uart_irq_rx_disable(dev);
            dtr_ready = false;
        }
        return false;
    }

    uint32_t dtr = 0;
    int ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
    
    bool terminal_connected = false;

    if (ret == 0) {
        terminal_connected = (dtr != 0);
    } else if (ret == -ENOTSUP || ret == -ENOSYS) {
        terminal_connected = true; 
    }

    if (terminal_connected && !dtr_ready) {
        LOG_INF("USB Terminal Opened (DTR High)");
        uart_irq_rx_enable(dev);
        dtr_ready = true;
    } else if (!terminal_connected && dtr_ready) {
        LOG_WRN("USB Terminal Closed (DTR Low)");
        uart_irq_rx_disable(dev);
        dtr_ready = false;
    }
    
    return dtr_ready;
}

void UsbCdcFacade::transmit(std::string_view data) noexcept {
    if (!isConnected()) {
        return;
    }
    std::size_t sent = 0;
    for (char c : data) {
        uart_poll_out(dev, c);
        if (++sent % TxChunkBytes == 0) {
            k_msleep(TxChunkGapMs);   // let the CDC TX ring buffer drain before sending more
        }
    }
}

void UsbCdcFacade::uartInterruptHandler(const device* dev, void* user_data) {
    auto* self = static_cast<UsbCdcFacade*>(user_data);
    uart_irq_update(dev);
    if (uart_irq_rx_ready(dev)) {
        bool discard_rest_of_burst = false;
        uint8_t c;
        while (uart_fifo_read(dev, &c, 1) == 1) {
            if (c == '\b' || c == 0x7F) {
                const std::size_t head = self->rx_head.load(std::memory_order_relaxed);
                const std::size_t tail = self->rx_tail.load(std::memory_order_acquire);
                
                if (head != tail) {
                    uart_poll_out(dev, '\b');
                    uart_poll_out(dev, ' ');
                    uart_poll_out(dev, '\b');
                    
                    self->rx_head.store((head + RX_RING_BUF_SIZE - 1) % RX_RING_BUF_SIZE, std::memory_order_release);
                }
                continue;
            }
            
            if (c == '\r' || c == '\n') {
                uart_poll_out(dev, '\r');
                uart_poll_out(dev, '\n');
            } else {
                uart_poll_out(dev, c);
            }

            if (discard_rest_of_burst) {
                self->dropped_bytes.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            const std::size_t head = self->rx_head.load(std::memory_order_relaxed);
            const std::size_t tail = self->rx_tail.load(std::memory_order_acquire);
            const std::size_t next_head = (head + 1) % RX_RING_BUF_SIZE;
            if (next_head != tail) {
                self->rx_buffer[head] = static_cast<char>(c);
                self->rx_head.store(next_head, std::memory_order_release);

                if (self->overflow_logged.exchange(false, std::memory_order_relaxed)) {
                    self->dropped_bytes.store(0, std::memory_order_relaxed);
                }
            } else {
                const std::size_t buffered = (head + RX_RING_BUF_SIZE - tail) % RX_RING_BUF_SIZE;
                self->rx_tail.store(head, std::memory_order_release);
                self->dropped_bytes.fetch_add(buffered + 1, std::memory_order_relaxed);
                self->overflow_count.fetch_add(1, std::memory_order_relaxed);
                discard_rest_of_burst = true;
                if (!self->overflow_logged.exchange(true, std::memory_order_relaxed)) {
                    LOG_WRN("USB RX overflow - input buffer flushed");
                }
            }
        }
    }
}

bool UsbCdcFacade::readLine(CommandBuffer& out_line) noexcept {
    const std::size_t head = rx_head.load(std::memory_order_acquire);
    std::size_t tail = rx_tail.load(std::memory_order_relaxed);
    if (head == tail) {
        return false;
    }
    std::size_t temp_tail = tail;
    std::size_t i = 0;
    bool found_eol = false;
    bool overlong = false;
    while (temp_tail != head) {
        const char c = rx_buffer[temp_tail];
        if (c == CR || c == LF) {
            found_eol = true;
            temp_tail = (temp_tail + 1) % RX_RING_BUF_SIZE;
            
            if (c == CR && temp_tail != head && rx_buffer[temp_tail] == LF) {
                temp_tail = (temp_tail + 1) % RX_RING_BUF_SIZE;
            }
            break;
        }
        if (i >= MAX_CMD_LEN - 1) {
            overlong = true;
        } else {
            out_line[i++] = c;
        }
        temp_tail = (temp_tail + 1) % RX_RING_BUF_SIZE;
    }
    if (!found_eol) {
        return false;
    }
    rx_tail.store(temp_tail, std::memory_order_release);
    if (overlong) {
        LOG_WRN("USB CLI command exceeded %zu bytes, discarded", MAX_CMD_LEN - 1);
        out_line[0] = '\0';
        return false;
    }
    out_line[i] = '\0';
    overflow_logged.store(false, std::memory_order_release);
    return true;
}
