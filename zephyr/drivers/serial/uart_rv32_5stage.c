/*
 * Custom RV32IMC 5-stage pipelined RISC-V CPU (Zynq-7020 FPGA soft-core)
 * UART driver, matching src/uart_mmio.v's register layout.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rv32_5stage_uart

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#include <zephyr/irq.h>
#include <zephyr/irq_multilevel.h>
#include <zephyr/irq_nextlevel.h>
#endif

/*
 * uart_mmio.v register map (all 32-bit, word-aligned, fixed offsets - not
 * independently relocatable, so these aren't taken from devicetree reg-names):
 *   TX_DATA   0x0  W   write triggers transmission of the low byte
 *   TX_STATUS 0x4  R   bit 0 = tx_ready
 *   RX_DATA   0x8  R   reading consumes the byte (rx_has_data clears)
 *   RX_STATUS 0xC  R   bit 0 = rx_has_data
 *
 * Baud rate is fixed in hardware at synthesis time (uart_tx.v/uart_rx.v
 * CLK_FREQ/BAUD_RATE parameters) - there is no runtime configuration
 * register, so current-speed in the devicetree is documentation only.
 */
#define UART_RV32_5STAGE_TX_DATA   0x0
#define UART_RV32_5STAGE_TX_STATUS 0x4
#define UART_RV32_5STAGE_RX_DATA   0x8
#define UART_RV32_5STAGE_RX_STATUS 0xC

#define UART_RV32_5STAGE_TX_READY_BIT   BIT(0)
#define UART_RV32_5STAGE_RX_HAS_DATA_BIT BIT(0)

struct uart_rv32_5stage_config {
	uint32_t base;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
	const struct device *intc_dev;
	uint32_t tx_irq;
	uint32_t rx_irq;
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
struct uart_rv32_5stage_data {
	uart_irq_callback_user_data_t cb;
	void *cb_data;
};
#endif

static void uart_rv32_5stage_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_rv32_5stage_config *cfg = dev->config;

	while (!(sys_read32(cfg->base + UART_RV32_5STAGE_TX_STATUS) &
		 UART_RV32_5STAGE_TX_READY_BIT)) {
	}

	sys_write32((uint32_t)c, cfg->base + UART_RV32_5STAGE_TX_DATA);
}

static int uart_rv32_5stage_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_rv32_5stage_config *cfg = dev->config;

	if (!(sys_read32(cfg->base + UART_RV32_5STAGE_RX_STATUS) &
	      UART_RV32_5STAGE_RX_HAS_DATA_BIT)) {
		return -1;
	}

	*c = (unsigned char)sys_read32(cfg->base + UART_RV32_5STAGE_RX_DATA);
	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_rv32_5stage_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct uart_rv32_5stage_config *cfg = dev->config;

	if (len < 1 || !(sys_read32(cfg->base + UART_RV32_5STAGE_TX_STATUS) &
			 UART_RV32_5STAGE_TX_READY_BIT)) {
		return 0;
	}

	sys_write32(tx_data[0], cfg->base + UART_RV32_5STAGE_TX_DATA);
	return 1;
}

static int uart_rv32_5stage_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_rv32_5stage_config *cfg = dev->config;

	if (size < 1 || !(sys_read32(cfg->base + UART_RV32_5STAGE_RX_STATUS) &
			  UART_RV32_5STAGE_RX_HAS_DATA_BIT)) {
		return 0;
	}

	rx_data[0] = (uint8_t)sys_read32(cfg->base + UART_RV32_5STAGE_RX_DATA);
	return 1;
}

static int uart_rv32_5stage_irq_tx_ready(const struct device *dev)
{
	const struct uart_rv32_5stage_config *cfg = dev->config;

	return (sys_read32(cfg->base + UART_RV32_5STAGE_TX_STATUS) &
		UART_RV32_5STAGE_TX_READY_BIT) != 0;
}

static int uart_rv32_5stage_irq_tx_complete(const struct device *dev)
{
	return uart_rv32_5stage_irq_tx_ready(dev);
}

/*
 * uart_mmio.v's tx_irq is edge-triggered (fires once on tx_ready's 0->1
 * transition, see its own header comment) - not the level-triggered "tx
 * ready, come get more data" signal most UART TX-empty interrupts are.
 * If TX is already idle (the common case: nothing queued yet) when a
 * caller enables the interrupt, no edge will ever occur and intc.v's
 * pending bit for this source would never latch, so the ISR (and thus
 * the registered callback) would never fire for that first byte. Prime
 * it here by invoking the callback once immediately if TX is already
 * ready, exactly as if the interrupt had already fired - callers that
 * loop on irq_tx_ready()/fifo_fill() from within their callback (the
 * standard Zephyr pattern) behave identically either way.
 */
static void uart_rv32_5stage_irq_tx_enable(const struct device *dev)
{
	const struct uart_rv32_5stage_config *cfg = dev->config;
	struct uart_rv32_5stage_data *data = dev->data;

	irq_enable_next_level(cfg->intc_dev, cfg->tx_irq);

	if (uart_rv32_5stage_irq_tx_ready(dev) && data->cb) {
		data->cb(dev, data->cb_data);
	}
}

static void uart_rv32_5stage_irq_tx_disable(const struct device *dev)
{
	const struct uart_rv32_5stage_config *cfg = dev->config;

	irq_disable_next_level(cfg->intc_dev, cfg->tx_irq);
}

static int uart_rv32_5stage_irq_rx_ready(const struct device *dev)
{
	const struct uart_rv32_5stage_config *cfg = dev->config;

	return (sys_read32(cfg->base + UART_RV32_5STAGE_RX_STATUS) &
		UART_RV32_5STAGE_RX_HAS_DATA_BIT) != 0;
}

static void uart_rv32_5stage_irq_rx_enable(const struct device *dev)
{
	const struct uart_rv32_5stage_config *cfg = dev->config;

	irq_enable_next_level(cfg->intc_dev, cfg->rx_irq);
}

static void uart_rv32_5stage_irq_rx_disable(const struct device *dev)
{
	const struct uart_rv32_5stage_config *cfg = dev->config;

	irq_disable_next_level(cfg->intc_dev, cfg->rx_irq);
}

static int uart_rv32_5stage_irq_is_pending(const struct device *dev)
{
	return uart_rv32_5stage_irq_tx_ready(dev) || uart_rv32_5stage_irq_rx_ready(dev);
}

static void uart_rv32_5stage_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void uart_rv32_5stage_irq_callback_set(const struct device *dev,
					       uart_irq_callback_user_data_t cb, void *cb_data)
{
	struct uart_rv32_5stage_data *data = dev->data;

	data->cb = cb;
	data->cb_data = cb_data;
}

/* Both TX-complete and RX-data-available share this one ISR, matching how
 * a single registered user callback is expected to check
 * uart_irq_tx_ready()/uart_irq_rx_ready() itself to see which fired.
 */
static void uart_rv32_5stage_isr(const struct device *dev)
{
	struct uart_rv32_5stage_data *data = dev->data;

	if (data->cb) {
		data->cb(dev, data->cb_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static DEVICE_API(uart, uart_rv32_5stage_api) = {
	.poll_in = uart_rv32_5stage_poll_in,
	.poll_out = uart_rv32_5stage_poll_out,
	.err_check = NULL,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_rv32_5stage_fifo_fill,
	.fifo_read = uart_rv32_5stage_fifo_read,
	.irq_tx_enable = uart_rv32_5stage_irq_tx_enable,
	.irq_tx_disable = uart_rv32_5stage_irq_tx_disable,
	.irq_tx_ready = uart_rv32_5stage_irq_tx_ready,
	.irq_tx_complete = uart_rv32_5stage_irq_tx_complete,
	.irq_rx_enable = uart_rv32_5stage_irq_rx_enable,
	.irq_rx_disable = uart_rv32_5stage_irq_rx_disable,
	.irq_rx_ready = uart_rv32_5stage_irq_rx_ready,
	.irq_is_pending = uart_rv32_5stage_irq_is_pending,
	.irq_update = uart_rv32_5stage_irq_update,
	.irq_callback_set = uart_rv32_5stage_irq_callback_set,
#endif
};

static int uart_rv32_5stage_init(const struct device *dev)
{
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	const struct uart_rv32_5stage_config *cfg = dev->config;

	cfg->irq_config_func(dev);
#else
	ARG_UNUSED(dev);
#endif
	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_RV32_5STAGE_IRQ_CONFIG_FUNC(inst)                                                   \
	static void uart_rv32_5stage_irq_config_##inst(const struct device *dev)                 \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                  \
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, tx), 0, uart_rv32_5stage_isr,               \
			    DEVICE_DT_INST_GET(inst), 0);                                         \
		IRQ_CONNECT(DT_INST_IRQN_BY_NAME(inst, rx), 0, uart_rv32_5stage_isr,               \
			    DEVICE_DT_INST_GET(inst), 0);                                         \
	}
#define UART_RV32_5STAGE_IRQ_CONFIG_INIT(inst) .irq_config_func = uart_rv32_5stage_irq_config_##inst,
/* DT_INST_IRQN_BY_NAME (not DT_INST_IRQ_BY_NAME(inst, name, irq), which
 * only returns the raw devicetree cell, e.g. plain 0/1) gives the actual
 * multi-level-encoded Zephyr IRQ number needed both for IRQ_CONNECT above
 * and to recover the local (level-2) bit position below.
 *
 * irq_from_level_2() (irq_multilevel.h) can't be used for that recovery
 * here: it goes through a union of bitfields, which isn't a constant
 * expression the C standard allows in a static initializer even though
 * the value is knowable at compile time. Same math, done as plain
 * preprocessor arithmetic instead.
 */
#define UART_RV32_5STAGE_IRQ_FROM_L2(irq)                                                        \
	((((irq) >> CONFIG_1ST_LEVEL_INTERRUPT_BITS) & BIT_MASK(CONFIG_2ND_LEVEL_INTERRUPT_BITS)) - 1)

#define UART_RV32_5STAGE_IRQ_CFG_FIELDS(inst)                                                    \
	.intc_dev = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), interrupt_parent)),               \
	.tx_irq = UART_RV32_5STAGE_IRQ_FROM_L2(DT_INST_IRQN_BY_NAME(inst, tx)),                    \
	.rx_irq = UART_RV32_5STAGE_IRQ_FROM_L2(DT_INST_IRQN_BY_NAME(inst, rx)),
#define UART_RV32_5STAGE_DATA_DEFINE(inst) static struct uart_rv32_5stage_data uart_rv32_5stage_data_##inst;
#define UART_RV32_5STAGE_DATA_PTR(inst) &uart_rv32_5stage_data_##inst
#else
#define UART_RV32_5STAGE_IRQ_CONFIG_FUNC(inst)
#define UART_RV32_5STAGE_IRQ_CONFIG_INIT(inst)
#define UART_RV32_5STAGE_IRQ_CFG_FIELDS(inst)
#define UART_RV32_5STAGE_DATA_DEFINE(inst)
#define UART_RV32_5STAGE_DATA_PTR(inst) NULL
#endif

#define UART_RV32_5STAGE_INIT(inst)                                                              \
	UART_RV32_5STAGE_IRQ_CONFIG_FUNC(inst)                                                    \
	UART_RV32_5STAGE_DATA_DEFINE(inst)                                                        \
	static const struct uart_rv32_5stage_config uart_rv32_5stage_cfg_##inst = {               \
		.base = DT_INST_REG_ADDR(inst),                                                   \
		UART_RV32_5STAGE_IRQ_CONFIG_INIT(inst)                                            \
		UART_RV32_5STAGE_IRQ_CFG_FIELDS(inst)                                             \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, uart_rv32_5stage_init, NULL, UART_RV32_5STAGE_DATA_PTR(inst),  \
			      &uart_rv32_5stage_cfg_##inst, PRE_KERNEL_1,                         \
			      CONFIG_SERIAL_INIT_PRIORITY, &uart_rv32_5stage_api);

DT_INST_FOREACH_STATUS_OKAY(UART_RV32_5STAGE_INIT)
