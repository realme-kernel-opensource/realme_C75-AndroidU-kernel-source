// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/arm-smccc.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <tracker.h>

static unsigned int gic_irq(struct irq_data *d)
{
	return d->hwirq;
}

static int virq_to_hwirq(unsigned int virq)
{
	struct irq_desc *desc;
	unsigned int hwirq = 0;

	desc = irq_to_desc(virq);
	if (desc)
		hwirq = gic_irq(&desc->irq_data);
	else
		WARN_ON(!desc);

	return hwirq;
}
#if IS_ENABLED(CONFIG_MTK_IRQ_DBG_LEGACY)
static int mt_irq_dump_status_buf(unsigned int irq, char *buf)
{
	unsigned int result;
	unsigned int hwirq;
	int num = 0;
	char *ptr = buf;
	struct arm_smccc_res res = {0};

	if (!ptr)
		return 0;

	hwirq = virq_to_hwirq(irq);

	if (!hwirq)
		return 0;

	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] hwirq = %d, virq: %d\n", hwirq, irq);
	if (num == PAGE_SIZE)
		goto OUT;

	arm_smccc_smc(MTK_SIP_KERNEL_GIC_DUMP, hwirq, 0, 0, 0, 0, 0, 0, &res);

	if (res.a0 == 0) {
		num += snprintf(ptr + num, PAGE_SIZE - num,
				"[mt gic dump] not allowed to dump!\n");
		if (num == PAGE_SIZE)
			goto OUT;

		return num;
	}

	/* get mask */
	result = res.a0 & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] enable = %d\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get group */
	result = (res.a0 >> 1) & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] group = %d\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get priority */
	result = (res.a0 >> 2) & 0xff;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] priority = %d\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get sensitivity */
	result = (res.a0 >> 10) & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] sensitivity = %x ", result);
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"(edge:0x1, level:0x0)\n");
	if (num == PAGE_SIZE)
		goto OUT;

	/* get pending status */
	result = (res.a0 >> 0xb) & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] pending = %x\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get active status */
	result = (res.a0 >> 0xc) & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] active status = %x\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get polarity */
	result = (res.a0 >> 0xd) & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] polarity = %x\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get target cpu mask */
	result = (res.a0 >> 0xe) & 0x3;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] tartget cpu mask = 0x%x\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	return num;

OUT:
	pr_info("[mt gic dump] buffer is full\n");

	return num;

}
#else
static int mt_irq_dump_status_buf(unsigned int irq, char *buf)
{
	unsigned int result;
	unsigned int hwirq;
	int num = 0;
	char *ptr = buf;
	struct arm_smccc_res res;

	if (!ptr)
		return 0;

	hwirq = virq_to_hwirq(irq);
	if (!hwirq)
		return 0;

	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] hwirq = %d, virq: %d\n", hwirq, irq);
	if (num == PAGE_SIZE)
		goto OUT;

	arm_smccc_smc(MTK_SIP_KERNEL_GIC_DUMP,
			0, hwirq, 0, 0, 0, 0, 0, &res);
	if (res.a0 == 0) {
		num += snprintf(ptr + num, PAGE_SIZE - num,
				"[mt gic dump] not allowed to dump!\n");
		if (num == PAGE_SIZE)
			goto OUT;

		return num;
	}

	/* get mask */
	result = res.a0 & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] enable = %d\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get group */
	result = (res.a0 >> 1) & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] group = %d\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get group modifier */
	result = (res.a0 >> 2) & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] group modifier = %d\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get priority */
	result = (res.a0 >> 3) & 0xff;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] priority = %d\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get sensitivity */
	result = (res.a0 >> 0xb) & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] sensitivity = %x ", result);
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"(edge:0x1, level:0x0)\n");
	if (num == PAGE_SIZE)
		goto OUT;

	/* get pending status */
	result = (res.a0 >> 0xc) & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] pending = %x\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get active status */
	result = (res.a0 >> 0xd) & 0x1;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] active status = %x\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	/* get target cpu mask */
	result = (res.a0 >> 0xe) & 0xffff;
	num += snprintf(ptr + num, PAGE_SIZE - num,
			"[mt gic dump] tartget cpu mask = 0x%x\n", result);
	if (num == PAGE_SIZE)
		goto OUT;

	return num;

OUT:
	pr_info("[mt gic dump] buffer is full\n");

	return num;

}
#endif

void mt_irq_dump_status(unsigned int irq)
{
	char *buf = kmalloc(PAGE_SIZE, GFP_ATOMIC);

	if (!buf)
		return;

	/* support GIC PPI/SPI only */
	if (mt_irq_dump_status_buf(irq, buf) > 0)
		pr_info("%s", buf);

	kfree(buf);

}
EXPORT_SYMBOL(mt_irq_dump_status);

void mt_tracker_dump(void)
{
	unsigned int i;
	unsigned int reg_value;
	unsigned int entry_valid;
	unsigned int entry_secure;
	unsigned int entry_id;
	unsigned int entry_address;

	pr_info("\n*************************** tracker log start ***************************\n");

	/* check if we got infra tracker timeout */
	if (readl(INFRA_TRACKER_CON) & (BUS_DBG_CON_TIMEOUT | DBG_SLV_CHECK)) {
		pr_info("\n*************************** INFRA ***************************\n");
		pr_info("infra tracker timeout (0x%08x)\n", readl(INFRA_TRACKER_CON));
		for (i = 0; i < INFRA_ENTRY_NUM; i++) {
			entry_address = readl(AR_TRACK_L(INFRA_TRACKER_CON, i));
			reg_value = readl(BUS_DBG_AR_TRACK_LOG(INFRA_TRACKER_CON, i));
			entry_valid = extract_n2mbits(reg_value, INFRA_VALID_S, INFRA_VALID_E);
			entry_secure = extract_n2mbits(reg_value, INFRA_SECURE_S, INFRA_SECURE_E);
			entry_id = extract_n2mbits(reg_value, INFRA_ID_S, INFRA_ID_E);
			if (entry_valid == 1)
				pr_info("read entry = %d, valid = 0x%x, non-secure = 0x%x,
					id = 0x%x, address = 0x%x\n",
					i, entry_valid, entry_secure, entry_id, entry_address);
		}
		for (i = 0; i < INFRA_ENTRY_NUM; i++) {
			entry_address = readl(AW_TRACK_L(INFRA_TRACKER_CON, i));
			reg_value = readl(BUS_DBG_AW_TRACK_LOG(INFRA_TRACKER_CON, i));
			entry_valid = extract_n2mbits(reg_value, INFRA_VALID_S, INFRA_VALID_E);
			entry_secure = extract_n2mbits(reg_value, INFRA_SECURE_S, INFRA_SECURE_E);
			entry_id = extract_n2mbits(reg_value, INFRA_ID_S, INFRA_ID_E);
			if (entry_valid == 1)
				pr_info("write entry = %d, valid = 0x%x, non-secure = 0x%x,
					id = 0x%x, address = 0x%x\n",
					i, entry_valid, entry_secure, entry_id, entry_address);
		}
	}

	pr_info("\n*************************** tracker log end ***************************\n");
}
EXPORT_SYMBOL(mt_tracker_dump);

static void mt_tracker_init(void)
{
	infra_tracker_base = ioremap(INFRA_TRACKER_BASE, 0x1000);
	if (!infra_tracker_base)
		pr_err("infra_tracker_base ioremap failed!\n");
}

static void mt_tracker_exit(void)
{
	if (infra_tracker_base)
		iounmap(infra_tracker_base);
}

static int __init irq_dbg_init(void)
{
	mt_tracker_init();
	return 0;
}

static __exit void irq_dbg_exit(void)
{
	mt_tracker_exit();
}

module_init(irq_dbg_init);
module_exit(irq_dbg_exit);


MODULE_DESCRIPTION("MediaTek IRQ dbg Driver");
MODULE_LICENSE("GPL v2");
