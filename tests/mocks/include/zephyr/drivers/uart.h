#pragma once
#include <stdint.h>
#define UART_LINE_CTRL_DTR 1
#define UART_ERROR_OVERRUN (1 << 0)
#define UART_ERROR_PARITY  (1 << 1)
#define UART_ERROR_FRAMING (1 << 2)
#define UART_ERROR_NOISE   (1 << 3)
extern "C" {
int uart_line_ctrl_get(const struct device *dev, uint32_t ctrl, uint32_t *val);
void uart_poll_out(const struct device *dev, unsigned char c);
int uart_irq_callback_user_data_set(const struct device *dev, void (*cb)(const struct device *, void *), void *user_data);
void uart_irq_rx_enable(const struct device *dev);
void uart_irq_rx_disable(const struct device *dev);
void uart_irq_tx_enable(const struct device *dev);
void uart_irq_tx_disable(const struct device *dev);
int uart_irq_update(const struct device *dev);
int uart_irq_rx_ready(const struct device *dev);
int uart_irq_tx_ready(const struct device *dev);
int uart_err_check(const struct device *dev);
int uart_fifo_read(const struct device *dev, uint8_t *tx_data, const int size);
int uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, const int size);
}
