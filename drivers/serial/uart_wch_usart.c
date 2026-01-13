/*
 * Copyright (c) 2024 Google LLC.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_usart

#include <errno.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/pm/device.h>

#include <hal_ch32fun.h>

struct usart_wch_config {
	USART_TypeDef *regs;
	const struct device *clock_dev;
	uint32_t current_speed;
	uint8_t parity;
	uint8_t clock_id;
	const struct pinctrl_dev_config *pin_cfg;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	void (*irq_config_func)(const struct device *dev);
#endif
	uint8_t rx_dma_channel;
#endif
	bool hw_flow_control;
};

#ifdef CONFIG_UART_ASYNC_API
struct uart_dma_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg;
	uint8_t *buffer;
	size_t buffer_length;
	size_t offset;
	volatile size_t counter;
	int32_t timeout;
	struct k_work_delayable timeout_work;
	bool enabled;
};
#endif

struct usart_wch_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;
	void *user_data;
#endif
#ifdef CONFIG_UART_ASYNC_API
	const struct device *uart_dev;
	uart_callback_t async_cb;
	void *async_user_data;
	struct uart_dma_stream dma_rx;
	struct uart_dma_stream dma_tx;
	uint8_t *rx_next_buffer;
	size_t rx_next_buffer_len;
#endif
};

#ifdef CONFIG_UART_ASYNC_API
static void usart_wch_async_evt_callback(struct usart_wch_data *data,
					  struct uart_event *evt)
{
	if (data->async_cb) {
		data->async_cb(data->uart_dev, evt, data->async_user_data);
	}
}

static inline void usart_wch_dma_tx_enable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	regs->CTLR3 |= USART_CTLR3_DMAT;
}

static inline void usart_wch_dma_tx_disable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	regs->CTLR3 &= ~USART_CTLR3_DMAT;
}

static inline void usart_wch_dma_rx_enable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	regs->CTLR3 |= USART_CTLR3_DMAR;
}

static inline void usart_wch_dma_rx_disable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	regs->CTLR3 &= ~USART_CTLR3_DMAR;
}

static void usart_wch_dma_tx_cb(const struct device *dma_dev, void *user_data,
			       uint32_t channel, int status)
{
	const struct device *uart_dev = user_data;
	struct usart_wch_data *data = uart_dev->data;
	struct uart_event evt;

	if (status < 0) {
		evt.type = UART_TX_ABORTED;
		evt.data.tx.buf = data->dma_tx.buffer;
		evt.data.tx.len = data->dma_tx.counter;
		usart_wch_async_evt_callback(data, &evt);
	} else {
		evt.type = UART_TX_DONE;
		evt.data.tx.buf = data->dma_tx.buffer;
		evt.data.tx.len = data->dma_tx.buffer_length;
		usart_wch_async_evt_callback(data, &evt);
	}

	data->dma_tx.buffer_length = 0;
	usart_wch_dma_tx_disable(uart_dev);
}

static void usart_wch_dma_rx_cb(const struct device *dma_dev, void *user_data,
			       uint32_t channel, int status)
{
	const struct device *uart_dev = user_data;
	struct usart_wch_data *data = uart_dev->data;
	struct uart_event evt;

	if (status < 0) {
		evt.type = UART_RX_STOPPED;
		evt.data.rx_stop.reason = UART_ERROR_OVERRUN;
		evt.data.rx_stop.data.buf = data->dma_rx.buffer;
		evt.data.rx_stop.data.len = data->dma_rx.counter;
		usart_wch_async_evt_callback(data, &evt);
		return;
	}

	/* Buffer full */
	evt.type = UART_RX_RDY;
	evt.data.rx.buf = data->dma_rx.buffer;
	evt.data.rx.len = data->dma_rx.buffer_length;
	evt.data.rx.offset = 0;
	usart_wch_async_evt_callback(data, &evt);

	if (data->rx_next_buffer) {
		evt.type = UART_RX_BUF_RELEASED;
		evt.data.rx_buf.buf = data->dma_rx.buffer;
		usart_wch_async_evt_callback(data, &evt);

		data->dma_rx.buffer = data->rx_next_buffer;
		data->dma_rx.buffer_length = data->rx_next_buffer_len;
		data->rx_next_buffer = NULL;
		data->rx_next_buffer_len = 0;

		/* Reload DMA */
		data->dma_rx.blk_cfg.block_size = data->dma_rx.buffer_length;
		data->dma_rx.blk_cfg.dest_address = (uint32_t)data->dma_rx.buffer;
		dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &data->dma_rx.dma_cfg);
		dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

		evt.type = UART_RX_BUF_REQUEST;
		usart_wch_async_evt_callback(data, &evt);
	} else {
		/* No next buffer, disable RX */
		usart_wch_dma_rx_disable(uart_dev);
		data->dma_rx.enabled = false;
		evt.type = UART_RX_DISABLED;
		usart_wch_async_evt_callback(data, &evt);
	}
}

static int usart_wch_async_callback_set(const struct device *dev,
					 uart_callback_t callback,
					 void *user_data)
{
	struct usart_wch_data *data = dev->data;

	data->async_cb = callback;
	data->async_user_data = user_data;

	return 0;
}

static int usart_wch_async_tx(const struct device *dev,
		const uint8_t *tx_data, size_t buf_size, int32_t timeout)
{
	struct usart_wch_data *data = dev->data;
	int ret;

	if (data->dma_tx.buffer_length != 0) {
		return -EBUSY;
	}

	data->dma_tx.buffer = (uint8_t *)tx_data;
	data->dma_tx.buffer_length = buf_size;
	data->dma_tx.timeout = timeout;

	data->dma_tx.blk_cfg.source_address = (uint32_t)tx_data;
	data->dma_tx.blk_cfg.block_size = buf_size;

	ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &data->dma_tx.dma_cfg);
	if (ret != 0) {
		return ret;
	}

	ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
	if (ret != 0) {
		return ret;
	}

	usart_wch_dma_tx_enable(dev);

	return 0;
}

static int usart_wch_async_tx_abort(const struct device *dev)
{
	struct usart_wch_data *data = dev->data;
	struct dma_status stat;
	struct uart_event evt;

	if (data->dma_tx.buffer_length == 0) {
		return -EFAULT;
	}

	dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
	usart_wch_dma_tx_disable(dev);

	if (dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &stat) == 0) {
		data->dma_tx.counter = data->dma_tx.buffer_length - stat.pending_length;
	}

	evt.type = UART_TX_ABORTED;
	evt.data.tx.buf = data->dma_tx.buffer;
	evt.data.tx.len = data->dma_tx.counter;
	usart_wch_async_evt_callback(data, &evt);

	data->dma_tx.buffer_length = 0;

	return 0;
}

static int usart_wch_async_rx_enable(const struct device *dev,
		uint8_t *rx_buf, size_t buf_size, int32_t timeout)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;
	struct usart_wch_data *data = dev->data;
	int ret;

	if (data->dma_rx.enabled) {
		return -EBUSY;
	}

	data->dma_rx.buffer = rx_buf;
	data->dma_rx.buffer_length = buf_size;
	data->dma_rx.counter = 0;
	data->dma_rx.timeout = timeout;

	data->dma_rx.blk_cfg.dest_address = (uint32_t)rx_buf;
	data->dma_rx.blk_cfg.block_size = buf_size;

	ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &data->dma_rx.dma_cfg);
	if (ret != 0) {
		return ret;
	}

	ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
	if (ret != 0) {
		return ret;
	}

	regs->STATR &= ~USART_STATR_RXNE;
	regs->CTLR1 |= USART_CTLR1_IDLEIE;

	usart_wch_dma_rx_enable(dev);
	data->dma_rx.enabled = true;

	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};
	usart_wch_async_evt_callback(data, &evt);

	return 0;
}

static int usart_wch_async_rx_buf_rsp(const struct device *dev, uint8_t *buf,
				       size_t len)
{
	struct usart_wch_data *data = dev->data;

	data->rx_next_buffer = buf;
	data->rx_next_buffer_len = len;

	return 0;
}

static int usart_wch_async_rx_disable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;
	struct usart_wch_data *data = dev->data;
	struct dma_status stat;
	struct uart_event evt;

	if (!data->dma_rx.enabled) {
		return -EFAULT;
	}

	regs->CTLR1 &= ~USART_CTLR1_IDLEIE;
	dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
	usart_wch_dma_rx_disable(dev);

	if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) == 0) {
		data->dma_rx.counter = data->dma_rx.buffer_length - stat.pending_length;
	}

	evt.type = UART_RX_RDY;
	evt.data.rx.buf = data->dma_rx.buffer;
	evt.data.rx.len = data->dma_rx.counter;
	evt.data.rx.offset = 0;
	usart_wch_async_evt_callback(data, &evt);

	evt.type = UART_RX_BUF_RELEASED;
	evt.data.rx_buf.buf = data->dma_rx.buffer;
	usart_wch_async_evt_callback(data, &evt);

	if (data->rx_next_buffer) {
		evt.type = UART_RX_BUF_RELEASED;
		evt.data.rx_buf.buf = data->rx_next_buffer;
		usart_wch_async_evt_callback(data, &evt);
		data->rx_next_buffer = NULL;
		data->rx_next_buffer_len = 0;
	}

	data->dma_rx.enabled = false;
	evt.type = UART_RX_DISABLED;
	usart_wch_async_evt_callback(data, &evt);

	return 0;
}
#endif /* CONFIG_UART_ASYNC_API */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
static void usart_wch_isr(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;
	struct usart_wch_data *data = dev->data;

#ifdef CONFIG_UART_ASYNC_API
	if (data->dma_rx.enabled && (regs->STATR & USART_STATR_IDLE)) {
		struct dma_status stat;
		struct uart_event evt;
		uint32_t count;

		/* Clear IDLE flag */
		volatile uint32_t dummy = regs->STATR;
		dummy = regs->DATAR;
		(void)dummy;

		if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) == 0) {
			count = data->dma_rx.buffer_length - stat.pending_length;
			if (count > data->dma_rx.counter) {
				evt.type = UART_RX_RDY;
				evt.data.rx.buf = data->dma_rx.buffer;
				evt.data.rx.len = count - data->dma_rx.counter;
				evt.data.rx.offset = data->dma_rx.counter;
				usart_wch_async_evt_callback(data, &evt);
				data->dma_rx.counter = count;
			}
		}
	}
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (data->cb) {
		data->cb(dev, data->user_data);
	}
#endif
}
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int usart_wch_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	if (!len || !(regs->STATR & USART_STATR_TXE)) {
		return 0;
	}

	regs->DATAR = tx_data[0];
	return 1;
}

static int usart_wch_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	if (!size || !(regs->STATR & USART_STATR_RXNE)) {
		return 0;
	}

	rx_data[0] = regs->DATAR;
	return 1;
}

static void usart_wch_irq_tx_enable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	regs->CTLR1 |= (USART_CTLR1_TXEIE | USART_CTLR1_TCIE);
}

static void usart_wch_irq_tx_disable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	regs->CTLR1 &= ~(USART_CTLR1_TXEIE | USART_CTLR1_TCIE);
}

static int usart_wch_irq_tx_ready(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	return (regs->STATR & USART_STATR_TXE) > 0;
}

static void usart_wch_irq_rx_enable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	regs->CTLR1 |= USART_CTLR1_RXNEIE;
}

static void usart_wch_irq_rx_disable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	regs->CTLR1 &= ~USART_CTLR1_RXNEIE;
}

static int usart_wch_irq_tx_complete(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	return (regs->STATR & USART_STATR_TC) > 0;
}

static int usart_wch_irq_rx_ready(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	return (regs->STATR & USART_STATR_RXNE) > 0;
}

static void usart_wch_irq_err_enable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	regs->CTLR1 |= USART_CTLR1_PEIE;
	regs->CTLR2 |= USART_CTLR2_LBDIE;
	regs->CTLR3 |= USART_CTLR3_EIE;
}

static void usart_wch_irq_err_disable(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	regs->CTLR1 &= ~USART_CTLR1_PEIE;
	regs->CTLR2 &= ~USART_CTLR2_LBDIE;
	regs->CTLR3 &= ~USART_CTLR3_EIE;
}

static int usart_wch_irq_is_pending(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;
	uint16_t ctlr1 = regs->CTLR1;
	uint16_t ctlr2 = regs->CTLR2;
	uint16_t ctlr3 = regs->CTLR3;
	uint16_t statr = regs->STATR;
	uint16_t stat_mask = 0;

	stat_mask |= (ctlr1 & USART_CTLR1_TXEIE) ? USART_STATR_TXE : 0;
	stat_mask |= (ctlr1 & USART_CTLR1_TCIE) ? USART_STATR_TC : 0;
	stat_mask |= (ctlr1 & USART_CTLR1_RXNEIE) ? USART_STATR_RXNE | USART_STATR_ORE : 0;
	stat_mask |= (ctlr1 & USART_CTLR1_IDLEIE) ? USART_STATR_IDLE : 0;
	stat_mask |= (ctlr1 & USART_CTLR1_PEIE) ? USART_STATR_PE : 0;

	stat_mask |= (ctlr2 & USART_CTLR2_LBDIE) ? USART_STATR_LBD : 0;
	stat_mask |=
		(ctlr3 & USART_CTLR3_EIE) ? USART_STATR_NE | USART_STATR_ORE | USART_STATR_FE : 0;
	stat_mask |= (ctlr3 & USART_CTLR3_CTSIE) ? USART_STATR_CTS : 0;

	return (statr & stat_mask) > 0;
}

static int usart_wch_irq_update(const struct device *dev)
{
	return 1;
}

static void usart_wch_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				       void *user_data)
{
	struct usart_wch_data *data = dev->data;

	data->cb = cb;
	data->user_data = user_data;
}
#endif

static int usart_wch_init(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	struct usart_wch_data *data = dev->data;
	USART_TypeDef *regs = config->regs;
	uint32_t ctlr1 = USART_CTLR1_TE | USART_CTLR1_RE | USART_CTLR1_UE;
	uint32_t clock_rate;
	clock_control_subsys_t clock_sys = (clock_control_subsys_t *)(uintptr_t)config->clock_id;
	uint32_t divn;
	int err;

	clock_control_on(config->clock_dev, clock_sys);

	err = clock_control_get_rate(config->clock_dev, clock_sys, &clock_rate);
	if (err != 0) {
		return err;
	}
	divn = (clock_rate + config->current_speed / 2) / config->current_speed;

	switch (config->parity) {
	case UART_CFG_PARITY_NONE:
		break;
	case UART_CFG_PARITY_ODD:
		ctlr1 |= USART_CTLR1_PCE | USART_CTLR1_PS;
		break;
	case UART_CFG_PARITY_EVEN:
		ctlr1 |= USART_CTLR1_PCE;
		break;
	default:
		return -EINVAL;
	}

	regs->BRR = divn;
	regs->CTLR1 = ctlr1;
	regs->CTLR2 = 0;
	regs->CTLR3 = config->hw_flow_control ? (USART_CTLR3_RTSE | USART_CTLR3_CTSE) : 0;

	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	config->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_ASYNC_API
	data->uart_dev = dev;
	if (config->dma_dev) {
		data->dma_tx.dma_dev = config->dma_dev;
		data->dma_tx.dma_channel = config->tx_dma_channel;
		data->dma_tx.dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		data->dma_tx.dma_cfg.source_data_size = DMA_ADDR_ADJ_INCREMENT;
		data->dma_tx.dma_cfg.dest_data_size = DMA_ADDR_ADJ_NO_CHANGE;
		data->dma_tx.dma_cfg.source_burst_length = 1;
		data->dma_tx.dma_cfg.dest_burst_length = 1;
		data->dma_tx.dma_cfg.dma_callback = usart_wch_dma_tx_cb;
		data->dma_tx.dma_cfg.user_data = (void *)dev;
		data->dma_tx.dma_cfg.block_count = 1;
		data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
		data->dma_tx.blk_cfg.dest_address = (uint32_t)&regs->DATAR;
		data->dma_tx.dma_cfg.channel_priority = 1;

		data->dma_rx.dma_dev = config->dma_dev;
		data->dma_rx.dma_channel = config->rx_dma_channel;
		data->dma_rx.dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		data->dma_rx.dma_cfg.source_data_size = DMA_ADDR_ADJ_NO_CHANGE;
		data->dma_rx.dma_cfg.dest_data_size = DMA_ADDR_ADJ_INCREMENT;
		data->dma_rx.dma_cfg.source_burst_length = 1;
		data->dma_rx.dma_cfg.dest_burst_length = 1;
		data->dma_rx.dma_cfg.dma_callback = usart_wch_dma_rx_cb;
		data->dma_rx.dma_cfg.user_data = (void *)dev;
		data->dma_rx.dma_cfg.block_count = 1;
		data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg;
		data->dma_rx.blk_cfg.source_address = (uint32_t)&regs->DATAR;
		data->dma_rx.dma_cfg.channel_priority = 1;
	}
#endif

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int usart_wch_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct usart_wch_config *config = dev->config;
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		config->regs->CTLR1 &= ~USART_CTLR1_UE;
		err = clock_control_off(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id);
		if (err < 0) {
			return err;
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		err = clock_control_on(config->clock_dev, (clock_control_subsys_t)(uintptr_t)config->clock_id);
		if (err < 0) {
			return err;
		}
		config->regs->CTLR1 |= USART_CTLR1_UE;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

static int usart_wch_poll_in(const struct device *dev, unsigned char *ch)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	if ((regs->STATR & USART_STATR_RXNE) == 0) {
		return -1;
	}

	*ch = regs->DATAR;
	return 0;
}

static void usart_wch_poll_out(const struct device *dev, unsigned char ch)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;

	while ((regs->STATR & USART_STATR_TXE) == 0) {
	}

	regs->DATAR = ch;
}

static int usart_wch_err_check(const struct device *dev)
{
	const struct usart_wch_config *config = dev->config;
	USART_TypeDef *regs = config->regs;
	uint32_t statr = regs->STATR;
	enum uart_rx_stop_reason errors = 0;

	if ((statr & USART_STATR_PE) != 0) {
		errors |= UART_ERROR_PARITY;
	}
	if ((statr & USART_STATR_LBD) != 0) {
		errors |= UART_BREAK;
	}
	if ((statr & USART_STATR_FE) != 0) {
		errors |= UART_ERROR_FRAMING;
	}
	if ((statr & USART_STATR_NE) != 0) {
		errors |= UART_ERROR_NOISE;
	}
	if ((statr & USART_STATR_ORE) != 0) {
		errors |= UART_ERROR_OVERRUN;
	}

	return errors;
}

static DEVICE_API(uart, usart_wch_driver_api) = {
	.poll_in = usart_wch_poll_in,
	.poll_out = usart_wch_poll_out,
	.err_check = usart_wch_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = usart_wch_fifo_fill,
	.fifo_read = usart_wch_fifo_read,
	.irq_tx_enable = usart_wch_irq_tx_enable,
	.irq_tx_disable = usart_wch_irq_tx_disable,
	.irq_tx_ready = usart_wch_irq_tx_ready,
	.irq_rx_enable = usart_wch_irq_rx_enable,
	.irq_rx_disable = usart_wch_irq_rx_disable,
	.irq_tx_complete = usart_wch_irq_tx_complete,
	.irq_rx_ready = usart_wch_irq_rx_ready,
	.irq_err_enable = usart_wch_irq_err_enable,
	.irq_err_disable = usart_wch_irq_err_disable,
	.irq_is_pending = usart_wch_irq_is_pending,
	.irq_update = usart_wch_irq_update,
	.irq_callback_set = usart_wch_irq_callback_set,
#endif
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = usart_wch_async_callback_set,
	.tx = usart_wch_async_tx,
	.tx_abort = usart_wch_async_tx_abort,
	.rx_enable = usart_wch_async_rx_enable,
	.rx_buf_rsp = usart_wch_async_rx_buf_rsp,
	.rx_disable = usart_wch_async_rx_disable,
#endif
};

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define USART_WCH_IRQ_HANDLER_DECL(idx)                                                            \
	static void usart_wch_irq_config_func_##idx(const struct device *dev);

#define USART_WCH_IRQ_HANDLER_FUNC(idx) .irq_config_func = usart_wch_irq_config_func_##idx,

#define USART_WCH_IRQ_HANDLER(idx)                                                                 \
	static void usart_wch_irq_config_func_##idx(const struct device *dev)                      \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), usart_wch_isr,        \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}
#else
#define USART_WCH_IRQ_HANDLER_DECL(idx)
#define USART_WCH_IRQ_HANDLER_FUNC(idx)
#define USART_WCH_IRQ_HANDLER(idx)
#endif

#define USART_WCH_INIT(idx)                                                                        \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	USART_WCH_IRQ_HANDLER_DECL(idx)                                                            \
	static struct usart_wch_data usart_wch_##idx##_data;                                       \
	static const struct usart_wch_config usart_wch_##idx##_config = {                          \
		.regs = (USART_TypeDef *)DT_INST_REG_ADDR(idx),                                    \
		.current_speed = DT_INST_PROP(idx, current_speed),                                 \
		.parity = DT_INST_ENUM_IDX(idx, parity),                                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(idx)),                              \
		.clock_id = DT_INST_CLOCKS_CELL(idx, id),                                          \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                    \
		USART_WCH_IRQ_HANDLER_FUNC(idx)                                                    \
		IF_ENABLED(CONFIG_UART_WCH_USART_DMA,                                              \
			   (.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, tx)),         \
			    .tx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(idx, tx, channel),         \
			    .rx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(idx, rx, channel),))      \
		.hw_flow_control = DT_INST_PROP_OR(idx, hw_flow_control, false) };           \
	PM_DEVICE_DT_INST_DEFINE(idx, usart_wch_pm_action);                                        \
	DEVICE_DT_INST_DEFINE(idx, &usart_wch_init, PM_DEVICE_DT_INST_GET(idx),                    \
			      &usart_wch_##idx##_data, &usart_wch_##idx##_config, PRE_KERNEL_1,    \
			      CONFIG_SERIAL_INIT_PRIORITY, &usart_wch_driver_api);                 \
	USART_WCH_IRQ_HANDLER(idx)

DT_INST_FOREACH_STATUS_OKAY(USART_WCH_INIT)
