/*
 * Second-level interrupt aggregator for src/intc.v: one shared ENABLE
 * mask register and one shared, edge-latched, write-1-to-clear PENDING
 * register (see src/intc.v's own header comment for the exact hardware
 * semantics), feeding the CPU's riscv,cpu-intc external interrupt line
 * (mip.MEIP / mie.MEIE, IRQ 11).
 *
 * Registered as a standard Zephyr multi-level-interrupt aggregator
 * (zephyr/irq_nextlevel.h + IRQ_PARENT_ENTRY_DEFINE), the same shape as
 * in-tree drivers/interrupt_controller/intc_dw.c. NOTE for whoever adds
 * the next interrupt-driven peripheral behind this controller: this
 * SoC's arch_irq_enable() (the generic soc/common/riscv-privileged one)
 * only special-cases level-2 IRQs for PLIC/CLIC/AIA, so calling the
 * generic irq_enable(irq) for a level-2 IRQ behind THIS controller will
 * not reach it - call irq_enable_next_level(DEVICE_DT_GET(DT_NODELABEL(intc1)), irq)
 * directly instead (see irq_enable_next_level()/irq_disable_next_level()
 * in zephyr/irq_nextlevel.h).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rv32_5stage_intc

#include <zephyr/device.h>
#include <zephyr/devicetree/interrupt_controller.h>
#include <zephyr/irq.h>
#include <zephyr/irq_nextlevel.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/sys/util.h>

/* Fixed in hardware (src/intc.v's NUM_SOURCES parameter): source 0 is
 * UART TX-complete, source 1 is UART RX-data, sources 2-7 are reserved.
 */
#define RV32_5STAGE_INTC_NUM_SOURCES 8

#define RV32_5STAGE_INTC_ENABLE_REG  0x0
#define RV32_5STAGE_INTC_PENDING_REG 0x4

typedef void (*rv32_5stage_intc_config_irq_t)(void);

struct rv32_5stage_intc_config {
	uint32_t base;
	uint32_t isr_table_offset;
	rv32_5stage_intc_config_irq_t config_func;
};

static void rv32_5stage_intc_isr(const struct device *dev)
{
	const struct rv32_5stage_intc_config *cfg = dev->config;
	uint32_t pending = sys_read32(cfg->base + RV32_5STAGE_INTC_PENDING_REG) &
			    sys_read32(cfg->base + RV32_5STAGE_INTC_ENABLE_REG);

	/* Clear every source being serviced up front (write-1-to-clear): a
	 * source that re-asserts while its own child ISR below is still
	 * running will simply re-latch on its next edge (src/intc.v does
	 * this unconditionally every cycle, independent of the bus write),
	 * rather than being lost by clearing after the child ISR runs.
	 */
	sys_write32(pending, cfg->base + RV32_5STAGE_INTC_PENDING_REG);

	while (pending) {
		uint32_t bit = find_lsb_set(pending) - 1;

		pending &= ~BIT(bit);
		_sw_isr_table[cfg->isr_table_offset + bit].isr(
			_sw_isr_table[cfg->isr_table_offset + bit].arg);
	}
}

static void rv32_5stage_intc_intr_enable(const struct device *dev, unsigned int irq)
{
	const struct rv32_5stage_intc_config *cfg = dev->config;
	uint32_t enable = sys_read32(cfg->base + RV32_5STAGE_INTC_ENABLE_REG);

	sys_write32(enable | BIT(irq), cfg->base + RV32_5STAGE_INTC_ENABLE_REG);
}

static void rv32_5stage_intc_intr_disable(const struct device *dev, unsigned int irq)
{
	const struct rv32_5stage_intc_config *cfg = dev->config;
	uint32_t enable = sys_read32(cfg->base + RV32_5STAGE_INTC_ENABLE_REG);

	sys_write32(enable & ~BIT(irq), cfg->base + RV32_5STAGE_INTC_ENABLE_REG);
}

static unsigned int rv32_5stage_intc_intr_get_state(const struct device *dev)
{
	const struct rv32_5stage_intc_config *cfg = dev->config;

	return sys_read32(cfg->base + RV32_5STAGE_INTC_ENABLE_REG) != 0;
}

static int rv32_5stage_intc_intr_get_line_state(const struct device *dev, unsigned int irq)
{
	const struct rv32_5stage_intc_config *cfg = dev->config;

	return (sys_read32(cfg->base + RV32_5STAGE_INTC_ENABLE_REG) & BIT(irq)) != 0;
}

static const struct irq_next_level_api rv32_5stage_intc_apis = {
	.intr_enable = rv32_5stage_intc_intr_enable,
	.intr_disable = rv32_5stage_intc_intr_disable,
	.intr_get_state = rv32_5stage_intc_intr_get_state,
	.intr_get_line_state = rv32_5stage_intc_intr_get_line_state,
};

static int rv32_5stage_intc_initialize(const struct device *dev)
{
	const struct rv32_5stage_intc_config *cfg = dev->config;

	/* Disable everything and clear any pending edges latched before the
	 * kernel got here, then wire up and enable our own (level-1) IRQ.
	 */
	sys_write32(0, cfg->base + RV32_5STAGE_INTC_ENABLE_REG);
	sys_write32(BIT_MASK(RV32_5STAGE_INTC_NUM_SOURCES), cfg->base + RV32_5STAGE_INTC_PENDING_REG);
	cfg->config_func();

	return 0;
}

#define RV32_5STAGE_INTC_INIT(inst)                                                              \
	static void rv32_5stage_intc_config_irq_##inst(void)                                     \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), 0, rv32_5stage_intc_isr,                          \
			    DEVICE_DT_INST_GET(inst), 0);                                         \
		irq_enable(DT_INST_IRQN(inst));                                                   \
	}                                                                                          \
	IRQ_PARENT_ENTRY_DEFINE(rv32_5stage_intc##inst, DEVICE_DT_INST_GET(inst),                \
				 DT_INST_IRQN(inst), INTC_INST_ISR_TBL_OFFSET(inst),              \
				 DT_INST_INTC_GET_AGGREGATOR_LEVEL(inst));                        \
                                                                                                   \
	static const struct rv32_5stage_intc_config rv32_5stage_intc_cfg_##inst = {              \
		.base = DT_INST_REG_ADDR(inst),                                                   \
		.isr_table_offset = INTC_INST_ISR_TBL_OFFSET(inst),                               \
		.config_func = rv32_5stage_intc_config_irq_##inst,                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, rv32_5stage_intc_initialize, NULL, NULL,                      \
			       &rv32_5stage_intc_cfg_##inst, PRE_KERNEL_1,                        \
			       CONFIG_INTC_RV32_5STAGE_INIT_PRIORITY, &rv32_5stage_intc_apis);

DT_INST_FOREACH_STATUS_OKAY(RV32_5STAGE_INTC_INIT)
