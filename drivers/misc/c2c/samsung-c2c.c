/*
 * Samsung C2C driver
 *
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Author: Kisang Lee <kisang80.lee@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/cma.h>
#ifdef ENABLE_C2CSTATE_TIMER
#include <linux/timer.h>
#endif
#ifdef CONFIG_C2C_IPC_ENABLE
#include <linux/vmalloc.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#endif
#include <asm/mach-types.h>

#include <mach/c2c.h>
#include <mach/regs-c2c.h>
#include <mach/regs-pmu.h>
#include <mach/pmu.h>

#include "samsung-c2c.h"

#ifdef ENABLE_C2CSTATE_TIMER
struct timer_list c2c_status_timer;

static void c2c_timer_func(unsigned long data)
{
	/* Check C2C state */
	static int current_state = 0xff;
	struct exynos_c2c_platdata *pdata = (struct exynos_c2c_platdata*)data;
	if (current_state != pdata->get_c2c_state()) {
		printk("[C2C]C2C state is chaged (0x%x --> 0x%x)\n",current_state, pdata->get_c2c_state());
		current_state = pdata->get_c2c_state();
	}
        c2c_status_timer.expires = jiffies + (HZ/5);
        add_timer(&c2c_status_timer);
}
#endif

void c2c_reset_ops(void)
{
	u32 set_clk = 0;

	if (c2c_con.opp_mode == C2C_OPP100)
		set_clk = c2c_con.clk_opp100 + 1;
	else if (c2c_con.opp_mode == C2C_OPP50)
		set_clk = c2c_con.clk_opp50 + 1;
	else if (c2c_con.opp_mode == C2C_OPP25)
		set_clk = c2c_con.clk_opp25 + 1;

	printk("[C2C] c2c_reset_ops()\n");
	clk_set_rate(c2c_con.c2c_sclk, set_clk * 1000000);
	c2c_set_func_clk(set_clk);

	/* First phase - C2C block reset */
	c2c_set_reset(C2C_CLEAR);
	c2c_set_reset(C2C_SET);
	/* Second phase - Clear clock gating */
	c2c_set_clock_gating(C2C_CLEAR);
	/* Third phase - Retention reg */
	c2c_writel(c2c_con.rtt_enableset, EXYNOS4_C2C_IRQ_EN_SET1);
	c2c_writel(set_clk, EXYNOS4_C2C_FCLK_FREQ);
	c2c_writel(set_clk, EXYNOS4_C2C_RX_MAX_FREQ);
	/* Last phase - Set clock gating */
	c2c_set_clock_gating(C2C_SET);
}

int c2c_open(struct inode *inode, struct file *filp)
{
	/* This function is not needed.(Test Function) */
	printk("[C2C] C2C chrdrv Opened.\n");

	return 0;
}

static long c2c_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	c2c_reset_ops();

	return 0;
}

struct file_operations c2c_fops = {
	.owner	= THIS_MODULE,
	.unlocked_ioctl	= c2c_ioctl,
	.open	= c2c_open,
};

static struct miscdevice char_dev = {
        .minor          = MISC_DYNAMIC_MINOR,
        .name           = C2C_DEV_NAME,
        .fops           = &c2c_fops
};

static int set_sharedmem(enum c2c_shrdmem_size size, u32 addr)
{
	c2c_dbg(KERN_INFO "[C2C] Set BaseAddr(0x%x) and Size(%d)\n", addr, 1 << (2 + size));

	/* Set DRAM Base Addr & Size */
	c2c_set_shdmem_size(size);
	c2c_set_base_addr((addr >> 22));

	return 0;
}

static irqreturn_t c2c_sscm0_irq(int irq, void *data)
{
	/* TODO : This function will be used other type boards */
	return IRQ_HANDLED;
}

static irqreturn_t c2c_sscm1_irq(int irq, void *data)
{
	/* TODO : It is just temporary code. It will be modified. */
	u32 raw_irq, latency_val, opp_val, req_clk;
	raw_irq = c2c_readl(EXYNOS4_C2C_IRQ_EN_STAT1);

#ifdef CONFIG_C2C_IPC_ENABLE
	if (raw_irq & 0x1) {
		c2c_dbg("IPC interrupt occured : GENO[0]\n");
		if (c2c_con.hd.handler)
			c2c_con.hd.handler(c2c_con.hd.handler);

		/* Interrupt Clear */
		c2c_writel(0x1, EXYNOS4_C2C_IRQ_EN_STAT1);
	}
#endif
	if ((raw_irq >> 27) & 1) { /* OPP Change */
		/*
		    If OPP clock is different with below table,
		    below codes should be changed.
		    OPP Change : Mhz : [29:28]
		    OPP(100) : 400 Mhz : 1 1
		    OPP(50) : 200 Mhz : 1 0
		    OPP(25) : 100 Mhz : 0 1
		*/
		opp_val = (c2c_readl(EXYNOS4_C2C_GENO_STATUS) >> 28) & 3;
		req_clk = 0;
		c2c_dbg("OPP interrupt occured (%d)\n", opp_val);

		if (opp_val == C2C_OPP100)
			req_clk = c2c_con.clk_opp100 + 1;
		else if (opp_val == C2C_OPP50)
			req_clk = c2c_con.clk_opp50 + 1;
		else if (opp_val == C2C_OPP25)
			req_clk = c2c_con.clk_opp25 + 1;

		if (opp_val == 0 || req_clk == 1) {
			printk("[C2C] This mode is not reserved in OPP mode.\n");
		} else {
			if (c2c_con.opp_mode > opp_val) { /* increase case */
				clk_set_rate(c2c_con.c2c_sclk, req_clk * 1000000);
				c2c_writel(req_clk, EXYNOS4_C2C_FCLK_FREQ);
				c2c_writel(req_clk, EXYNOS4_C2C_RX_MAX_FREQ);
				c2c_set_func_clk(req_clk);
			} else if (c2c_con.opp_mode < opp_val) { /* decrease case */
				c2c_writel(req_clk, EXYNOS4_C2C_RX_MAX_FREQ);
				clk_set_rate(c2c_con.c2c_sclk, req_clk * 1000000);
				c2c_writel(req_clk, EXYNOS4_C2C_FCLK_FREQ);
				c2c_set_func_clk(req_clk);
			} else{
				printk("Requested same OPP mode\n");
			}
			c2c_con.opp_mode = opp_val;
		}

		/* Interrupt Clear */
		c2c_writel((0x1 << 27), EXYNOS4_C2C_IRQ_EN_STAT1);
	}
	if ((raw_irq >> 26) & 1) { /* Memory I/F latency change */
		latency_val = (c2c_readl(EXYNOS4_C2C_GENO_STATUS) >> 30) & 3;
		switch(latency_val) {
		case 3:
			c2c_dbg("[C2C] Set Min latency\n");
			exynos4_c2c_request_pwr_mode(MIN_LATENCY);
			break;
		case 1:
			c2c_dbg("[C2C] Set Short latency\n");
			exynos4_c2c_request_pwr_mode(SHORT_LATENCY);
			break;
		case 0:
			c2c_dbg("[C2C] Set Max latency\n");
			exynos4_c2c_request_pwr_mode(MAX_LATENCY);
			break;
		}
		/* Interrupt Clear */
		c2c_writel((0x1 << 26), EXYNOS4_C2C_IRQ_EN_STAT1);
	}

	return IRQ_HANDLED;
}

void set_c2c_device(struct platform_device *pdev)
{
	struct exynos_c2c_platdata *pdata = pdev->dev.platform_data;
	u32 default_clk;

	c2c_con.rx_width = pdata->rx_width;
	c2c_con.tx_width = pdata->tx_width;
	c2c_con.clk_opp100 = pdata->clk_opp100;
	c2c_con.clk_opp50 = pdata->clk_opp50;
	c2c_con.clk_opp25 = pdata->clk_opp25;
	c2c_con.opp_mode = pdata->default_opp_mode;
#ifdef CONFIG_C2C_IPC_ENABLE
	c2c_con.shd_pages = NULL;
	c2c_con.hd.data = NULL;
	c2c_con.hd.handler = NULL;
#endif
	c2c_con.c2c_sclk = clk_get(&pdev->dev, "sclk_c2c");
	c2c_con.c2c_aclk = clk_get(&pdev->dev, "aclk_c2c");

	/* Set clock to default mode */
	if (c2c_con.opp_mode == C2C_OPP100)
		default_clk = c2c_con.clk_opp100 + 1;
	else if (c2c_con.opp_mode == C2C_OPP50)
		default_clk = c2c_con.clk_opp50 + 1;
	else if (c2c_con.opp_mode == C2C_OPP25)
		default_clk = c2c_con.clk_opp25 + 1;
	else {
		printk("[C2C] Default OPP mode is not selected.\n");
		c2c_con.opp_mode = C2C_OPP50;
		default_clk = c2c_con.clk_opp50;
	}

	clk_set_rate(c2c_con.c2c_sclk, default_clk  * 1000000);
	clk_set_rate(c2c_con.c2c_aclk, ((default_clk / 2) + 1) * 1000000);

	c2c_dbg("[C2C] Get C2C sclk rate : %ld\n", clk_get_rate(c2c_con.c2c_sclk));
	c2c_dbg("[C2C] Get C2C aclk rate : %ld\n", clk_get_rate(c2c_con.c2c_aclk));
	if (pdata->setup_gpio)
		pdata->setup_gpio(pdata->rx_width, pdata->tx_width);

	set_sharedmem(pdata->shdmem_size, pdata->shdmem_addr);

	/* Set SYSREG to memdone */
	c2c_set_memdone(C2C_SET);
	c2c_set_clock_gating(C2C_CLEAR);

	/* Set C2C clock register to OPP50 */
	c2c_writel(default_clk, EXYNOS4_C2C_FCLK_FREQ);
	c2c_writel(default_clk, EXYNOS4_C2C_RX_MAX_FREQ);
	c2c_set_func_clk(default_clk);

	/* Set C2C buswidth */
	c2c_writel(((pdata->rx_width << 4) |(pdata->tx_width)), EXYNOS4_C2C_PORTCONFIG);
	c2c_set_tx_buswidth(pdata->tx_width);
	c2c_set_rx_buswidth(pdata->rx_width);

	/* Enable all of GENI/O Interrupt */
	c2c_writel(0x8000000, EXYNOS4_C2C_IRQ_EN_SET1);
	c2c_con.rtt_enableset = 0x8000000;

	exynos4_c2c_request_pwr_mode(MAX_LATENCY);

	c2c_writel(0x8000000, EXYNOS4_C2C_GENO_INT);
	c2c_writel(0x8000000, EXYNOS4_C2C_GENO_LEVEL);

	c2c_dbg("[C2C] Port Config : 0x%x\n", c2c_readl(EXYNOS4_C2C_PORTCONFIG));
	c2c_dbg("[C2C] FCLK_FREQ register : %d\n", c2c_readl(EXYNOS4_C2C_FCLK_FREQ));
	c2c_dbg("[C2C] RX_MAX_FREQ register : %d\n", c2c_readl(EXYNOS4_C2C_RX_MAX_FREQ));
	c2c_dbg("[C2C] IRQ_EN_SET1 register : 0x%x\n", c2c_readl(EXYNOS4_C2C_IRQ_EN_SET1));

	c2c_set_clock_gating(C2C_SET);
}

#ifdef CONFIG_C2C_IPC_ENABLE
volatile void __iomem *c2c_request_cp_region(unsigned int cp_addr,
		unsigned int size)
{
	dma_addr_t phy_cpmem;

	phy_cpmem = cma_alloc(c2c_con.c2c_dev, "c2c_shdmem", size, 0);
	if (IS_ERR_VALUE(phy_cpmem)) {
		printk(KERN_ERR "C2C CMA Alloc Error!!!");
		return -ENOMEM;
	}

	return phys_to_virt(phy_cpmem);
}
EXPORT_SYMBOL(c2c_request_cp_region);

void c2c_release_cp_region(void *rgn)
{
	dma_addr_t phy_cpmem;

	phy_cpmem = virt_to_phys(rgn);

	cma_free(phy_cpmem);
}
EXPORT_SYMBOL(c2c_release_cp_region);

volatile void __iomem *c2c_request_sh_region(unsigned int sh_addr,
		unsigned int size)
{
	int i;
	struct page **pages;
	void *pv;

	pages = kmalloc((size >> PAGE_SHIFT) * sizeof(*pages), GFP_KERNEL);
	for (i = 0; i < (size >> PAGE_SHIFT); i++) {
		pages[i] = phys_to_page(sh_addr);
		sh_addr += PAGE_SIZE;
	}

	c2c_con.shd_pages = (void *)pages;

	pv = vmap(pages, size >> PAGE_SHIFT, VM_MAP, pgprot_noncached(PAGE_KERNEL));

	return (void __iomem *)pv;
}
EXPORT_SYMBOL(c2c_request_sh_region);

void c2c_release_sh_region(void *rgn)
{
	vunmap(rgn);
	kfree(c2c_con.shd_pages);
	c2c_con.shd_pages = NULL;
}
EXPORT_SYMBOL(c2c_release_sh_region);

int c2c_register_handler(void (*handler)(void *), void *data)
{
	if (!handler)
		return -EINVAL;

	c2c_con.hd.data = data;
	c2c_con.hd.handler = handler;

	c2c_reset_interrupt();

	return 0;
}
EXPORT_SYMBOL(c2c_register_handler);

int c2c_unregister_handler(void (*handler)(void *))
{
	if (!handler)
		return -EINVAL;

	if (c2c_con.hd.data == handler) {
		c2c_con.hd.data = NULL;
		c2c_con.hd.handler = NULL;
	}

	return 0;
}
EXPORT_SYMBOL(c2c_unregister_handler);

void c2c_send_interrupt(void) {
	c2c_writel(c2c_readl(EXYNOS4_C2C_GENI_CONTROL) ^ 0x1,
			EXYNOS4_C2C_GENI_CONTROL);
}
EXPORT_SYMBOL(c2c_send_interrupt);

void c2c_reset_interrupt(void) {
	c2c_writel(c2c_readl(EXYNOS4_C2C_IRQ_EN_SET1) | 0x1,
			EXYNOS4_C2C_IRQ_EN_SET1);
	c2c_con.rtt_enableset |= 0x1;
}
EXPORT_SYMBOL(c2c_reset_interrupt);
#endif

static int __devinit samsung_c2c_probe(struct platform_device *pdev)
{
	struct exynos_c2c_platdata *pdata = pdev->dev.platform_data;
	struct resource *res = NULL;
	struct resource *res1 = NULL;
	int sscm_irq0, sscm_irq1, err;
	err = 0;

	c2c_con.c2c_dev = &pdev->dev;

	/* resource for AP's SSCM region */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource defined(AP's SSCM)\n");
		return -ENOENT;
	}

	res1 = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res1) {
		dev_err(&pdev->dev, "failded to request memory resource(AP)\n");
		return -ENOENT;
	}

	pdata->ap_sscm_addr = ioremap(res->start, resource_size(res));
	if (!pdata->ap_sscm_addr) {
		dev_err(&pdev->dev, "failded to request memory resource(AP)\n");
		goto release_ap_sscm;
	}

	c2c_con.ap_sscm_addr = pdata->ap_sscm_addr;

	/* resource for CP's SSCM region */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource defined(AP's SSCM)\n");
		goto unmap_ap_sscm;
	}
	res1 = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res1) {
		dev_err(&pdev->dev, "failded to request memory resource(AP)\n");
		goto unmap_ap_sscm;
	}
	pdata->cp_sscm_addr = ioremap(res->start, resource_size(res));
	if (!pdata->ap_sscm_addr) {
		dev_err(&pdev->dev, "failded to request memory resource(AP)\n");
		goto release_cp_sscm;
	}

	c2c_con.cp_sscm_addr = pdata->cp_sscm_addr;

	/* Request IRQ */
	sscm_irq0 = platform_get_irq(pdev, 0);
	if (sscm_irq0 < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		goto unmap_cp_sscm;
	}
	err = request_irq(sscm_irq0, c2c_sscm0_irq, 0, pdev->name, pdev);
	if (err) {
		dev_err(&pdev->dev, "Can't request SSCM0 IRQ\n");
		goto unmap_cp_sscm;
	}
	/* SSCM0 irq will be only used from master(CP) device */
	disable_irq(sscm_irq0);

	sscm_irq1 = platform_get_irq(pdev, 1);
	if (sscm_irq1 < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		goto release_sscm_irq0;
	}
	err = request_irq(sscm_irq1, c2c_sscm1_irq, 1, pdev->name, pdev);
	if (err) {
		dev_err(&pdev->dev, "Can't request SSCM1 IRQ\n");
		goto release_sscm_irq0;
	}

	set_c2c_device(pdev);

	/* It is test code for C2C link.
	if (pdata->set_cprst)
		pdata->set_cprst();
	*/

	err = misc_register(&char_dev);
	if (err) {
		printk(KERN_ERR "[C2C] Can't register chrdev!\n");
		goto release_sscm_irq0;
	}

#ifdef ENABLE_C2CSTATE_TIMER
	/* Timer for debugging to check C2C state */
	init_timer(&c2c_status_timer);
	c2c_status_timer.expires = jiffies + HZ;
	c2c_status_timer.data = (unsigned long)pdata;
	c2c_status_timer.function = &c2c_timer_func;
	add_timer(&c2c_status_timer);
#endif

	return 0;

release_sscm_irq0:
	free_irq(sscm_irq0, pdev);

unmap_cp_sscm:
	iounmap(pdata->cp_sscm_addr);

release_cp_sscm:
	release_mem_region(res1->start, resource_size(res1));

unmap_ap_sscm:
	iounmap(pdata->ap_sscm_addr);

release_ap_sscm:
	release_mem_region(res->start, resource_size(res));

	return err;
}

static int __devexit samsung_c2c_remove(struct platform_device *pdev)
{
	/* TODO */
	return 0;
}

#ifdef CONFIG_PM
static int samsung_c2c_suspend(struct platform_device *dev, pm_message_t pm)
{
	/* TODO */
	return 0;
}

static int samsung_c2c_resume(struct platform_device *dev)
{
	struct exynos_c2c_platdata *pdata = dev->dev.platform_data;
	/* Set SYSREG */
	set_sharedmem(pdata->shdmem_size, pdata->shdmem_addr);
	c2c_set_memdone(C2C_SET);

	return 0;
}
#else
#define samsung_c2c_suspend NULL
#define samsung_c2c_resume NULL
#endif

static struct platform_driver samsung_c2c_driver = {
	.probe		= samsung_c2c_probe,
	.remove		= __devexit_p(samsung_c2c_remove),
	.suspend	= samsung_c2c_suspend,
	.resume		= samsung_c2c_resume,
	.driver		= {
		.name	= "samsung-c2c",
		.owner	= THIS_MODULE,
	},
};

static int __init samsung_c2c_init(void)
{
	return platform_driver_register(&samsung_c2c_driver);
}
module_init(samsung_c2c_init);

static void __exit samsung_c2c_exit(void)
{
	platform_driver_unregister(&samsung_c2c_driver);
}
module_exit(samsung_c2c_exit);

MODULE_DESCRIPTION("Samsung c2c driver");
MODULE_AUTHOR("Kisang Lee <kisang80.lee@samsung.com>");
MODULE_LICENSE("GPL");