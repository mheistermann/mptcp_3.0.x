/*
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Author: Yulgon Kim <yulgon.kim@samsung.com>
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/regs-pmu.h>
#include <mach/regs-pmu5.h>
#include <mach/regs-usb-phy.h>
#include <plat/cpu.h>
#include <plat/usb-phy.h>
#include <plat/cpu.h>
#include <plat/regs-usb3-exynos-drd-phy.h>

#define ETC6PUD		(S5P_VA_GPIO2 + 0x228)
#define USB_CFG		(S3C_VA_SYS + 0x21C)

#define PHY_ENABLE	(1 << 0)
#define PHY_DISABLE	(0)

enum usb_phy_type {
	USB_PHY		= (0x1 << 0),
	USB_PHY0	= (0x1 << 0),
	USB_PHY1	= (0x1 << 1),
	USB_PHY_HSIC0	= (0x1 << 1),
	USB_PHY_HSIC1	= (0x1 << 2),
};

struct exynos4_usb_phy {
	u8 phy0_usage;
	u8 phy1_usage;
	u8 phy2_usage;
};

static struct exynos4_usb_phy usb_phy_control;
static atomic_t host_usage;
static DEFINE_SPINLOCK(phy_lock);

static void exynos4_usb_mux_change(int val)
{
	writel(val, USB_CFG);
}

static int exynos4_usb_host_phy_is_on(void)
{
	return (readl(EXYNOS4_PHYPWR) & PHY1_STD_ANALOG_POWERDOWN) ? 0 : 1;
}

static int exynos4_usb_device_phy_is_on(void)
{
	return (readl(EXYNOS4_PHYPWR) & PHY0_ANALOG_POWERDOWN) ? 0 : 1;
}

static int exynos5_usb_host_phy20_is_on(void)
{
	return (readl(EXYNOS5_PHY_HOST_CTRL0) & HOST_CTRL0_PHYSWRSTALL) ? 0 : 1;
}

static u32 exynos4_usb_phy_set_clock(struct platform_device *pdev)
{
	struct clk *xusbxti_clk;
	u32 phyclk = readl(EXYNOS4_PHYCLK);

	xusbxti_clk = clk_get(&pdev->dev, "xusbxti");
	if (IS_ERR(xusbxti_clk)) {
		printk(KERN_ERR"Failed to get xusbxti clock\n");
		return PTR_ERR(xusbxti_clk);
	}

	if (soc_is_exynos4210()) {
		phyclk &= ~(EXYNOS4210_PHY0_ID_PULLUP |
				EXYNOS4210_CLKSEL_MASK);
		switch (clk_get_rate(xusbxti_clk)) {
		case 12 * MHZ:
			phyclk |= EXYNOS4210_CLKSEL_12M;
			break;
		case 24 * MHZ:
			phyclk |= EXYNOS4210_CLKSEL_24M;
			break;
		default:
		case 48 * MHZ:
			phyclk |= EXYNOS4210_CLKSEL_48M;
			/* default reference clock */
			break;
		}
	} else {
		phyclk &= ~(EXYNOS4212_PHY0_ID_PULLUP |
				EXYNOS4212_CLKSEL_MASK);
		switch (clk_get_rate(xusbxti_clk)) {
		case 96 * 100000:
			phyclk |= EXYNOS4212_CLKSEL_9600K;
			break;
		case 10 * MHZ:
			phyclk |= EXYNOS4212_CLKSEL_10M;
			break;
		case 12 * MHZ:
			phyclk |= EXYNOS4212_CLKSEL_12M;
			break;
		case 192 * 100000:
			phyclk |= EXYNOS4212_CLKSEL_19200K;
			break;
		case 20 * MHZ:
			phyclk |= EXYNOS4212_CLKSEL_20M;
			break;
		default:
		case 24 * MHZ:
			/* default reference clock */
			phyclk |= EXYNOS4212_CLKSEL_24M;
			break;
		}
	}

	phyclk |= PHY1_COMMON_ON_N;
	clk_put(xusbxti_clk);

	return phyclk;
}

static void exynos4_usb_phy_control(enum usb_phy_type phy_type , int on)
{
	spin_lock(&phy_lock);

	if (soc_is_exynos4210()) {
		if (phy_type & USB_PHY0) {
			if (on == PHY_ENABLE && (usb_phy_control.phy0_usage++) == 0)
				writel(PHY_ENABLE, S5P_USBOTG_PHY_CONTROL);
			else if (on == PHY_DISABLE && (--usb_phy_control.phy0_usage) == 0)
				writel(PHY_DISABLE, S5P_USBOTG_PHY_CONTROL);
		}
		if (phy_type & USB_PHY1) {
			if (on == PHY_ENABLE && (usb_phy_control.phy1_usage++) == 0)
				writel(PHY_ENABLE, S5P_USBHOST_PHY_CONTROL);
			else if (on == PHY_DISABLE && (--usb_phy_control.phy1_usage) == 0)
				writel(PHY_DISABLE, S5P_USBHOST_PHY_CONTROL);
		}
	} else if (soc_is_exynos4212() | soc_is_exynos4412()){
		if (phy_type & USB_PHY) {
			if (on == PHY_ENABLE && (usb_phy_control.phy0_usage++) == 0)
				writel(PHY_ENABLE, S5P_USB_PHY_CONTROL);
			else if (on == PHY_DISABLE && (--usb_phy_control.phy0_usage) == 0)
				writel(PHY_DISABLE, S5P_USB_PHY_CONTROL);
		}
#ifdef CONFIG_USB_S5P_HSIC0
		if (phy_type & USB_PHY_HSIC0) {
			if (on == PHY_ENABLE && (usb_phy_control.phy1_usage++) == 0)
				writel(PHY_ENABLE, S5P_HSIC_1_PHY_CONTROL);
			else if (on == PHY_DISABLE && (--usb_phy_control.phy1_usage) == 0)
				writel(PHY_DISABLE, S5P_HSIC_1_PHY_CONTROL);
		}
#endif
#ifdef CONFIG_USB_S5P_HSIC1
		if (phy_type & USB_PHY_HSIC1) {
			if (on == PHY_ENABLE && (usb_phy_control.phy2_usage++) == 0)
				writel(PHY_ENABLE, S5P_HSIC_2_PHY_CONTROL);
			else if (on == PHY_DISABLE && (--usb_phy_control.phy2_usage) == 0)
				writel(PHY_DISABLE, S5P_HSIC_2_PHY_CONTROL);
		}
#endif
	} else {
		if (phy_type & USB_PHY0) {
			if (on == PHY_ENABLE && (usb_phy_control.phy0_usage++) == 0)
				writel(PHY_ENABLE, EXYNOS5_USBDEV_PHY_CONTROL);
			else if (on == PHY_DISABLE && (--usb_phy_control.phy0_usage) == 0)
				writel(PHY_DISABLE, EXYNOS5_USBDEV_PHY_CONTROL);
		}

		if (phy_type & USB_PHY1) {
			if (on == PHY_ENABLE && (usb_phy_control.phy1_usage++) == 0)
				writel(PHY_ENABLE, EXYNOS5_USBHOST_PHY_CONTROL);
			else if (on == PHY_DISABLE && (--usb_phy_control.phy1_usage) == 0)
				writel(PHY_DISABLE, EXYNOS5_USBHOST_PHY_CONTROL);
		}
	}

	spin_unlock(&phy_lock);
}

static int exynos4_usb_phy0_init(struct platform_device *pdev)
{
	struct clk *otg_clk;
	u32 phypwr;
	u32 phyclk;
	u32 rstcon;
	int err;

	exynos4_usb_phy_control(USB_PHY0, PHY_ENABLE);

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		printk(KERN_ERR"Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}

	/* set clock frequency for PLL */
	phyclk = exynos4_usb_phy_set_clock(pdev);
	writel(phyclk, EXYNOS4_PHYCLK);

	/* USB MUX change from Host to Device */
	if (soc_is_exynos4212() || soc_is_exynos4412())
		exynos4_usb_mux_change(0);

	/* set to normal of PHY0 */
	phypwr = readl(EXYNOS4_PHYPWR) & ~PHY0_NORMAL_MASK;
	writel(phypwr, EXYNOS4_PHYPWR);

	/* reset all ports of both PHY and Link */
	rstcon = readl(EXYNOS4_RSTCON) | PHY0_SWRST_MASK;
	writel(rstcon, EXYNOS4_RSTCON);
	udelay(10);
	rstcon &= ~PHY0_SWRST_MASK;
	writel(rstcon, EXYNOS4_RSTCON);

	clk_put(otg_clk);
	return 0;
}

static int exynos4_usb_phy0_exit(struct platform_device *pdev)
{
	struct clk *otg_clk;

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	/* unset to normal of PHY0 */
	writel((readl(EXYNOS4_PHYPWR) | PHY0_NORMAL_MASK),
			EXYNOS4_PHYPWR);

	exynos4_usb_phy_control(USB_PHY0, PHY_DISABLE);

	/* USB MUX change from Device to Host */
	if (soc_is_exynos4212() || soc_is_exynos4412())
		exynos4_usb_mux_change(1);
	clk_disable(otg_clk);
	clk_put(otg_clk);

	return 0;
}

int exynos4_check_usb_op(void)
{
	static struct clk *otg_clk;
	unsigned long flags;
	u32 phypwr;
	u32 op = 1;

	if (!otg_clk) {
		otg_clk = clk_get(NULL, "usbotg");
		if (IS_ERR(otg_clk)) {
			printk(KERN_ERR"Failed to get otg clock\n");
			otg_clk = NULL;
			return 0;
		}
	}
	clk_enable(otg_clk);

	local_irq_save(flags);
	phypwr = readl(EXYNOS4_PHYPWR);

	/*If USB Device is power on,  */
	if (exynos4_usb_device_phy_is_on()) {
		op = 1;
		goto done;
	} else if (!exynos4_usb_host_phy_is_on()) {
		op = 0;
		goto done;
	}

	/*If USB Device & Host is suspended,  */
	if (soc_is_exynos4210()) {
		if (phypwr & (PHY1_STD_FORCE_SUSPEND
			| EXYNOS4210_HSIC0_FORCE_SUSPEND
			| EXYNOS4210_HSIC1_FORCE_SUSPEND)) {
			writel(readl(EXYNOS4_PHYPWR)
				| PHY1_STD_ANALOG_POWERDOWN,
				EXYNOS4_PHYPWR);
			exynos4_usb_phy_control(USB_PHY1, PHY_DISABLE);

			op = 0;
		}
	} else {
		if (phypwr & (PHY1_STD_FORCE_SUSPEND
			| EXYNOS4212_HSIC0_FORCE_SUSPEND
			| EXYNOS4212_HSIC1_FORCE_SUSPEND)) {
			writel(readl(EXYNOS4_PHYPWR)
				| PHY1_STD_ANALOG_POWERDOWN
				| EXYNOS4212_HSIC0_ANALOG_POWERDOWN
				| EXYNOS4212_HSIC1_ANALOG_POWERDOWN,
				EXYNOS4_PHYPWR);
			exynos4_usb_phy_control(USB_PHY
				| USB_PHY_HSIC0
				| USB_PHY_HSIC1,
				PHY_DISABLE);

			op = 0;
		}
	}
done:
	local_irq_restore(flags);
	clk_disable(otg_clk);

	return op;
}

static int exynos4_usb_phy1_suspend(struct platform_device *pdev)
{
	struct clk *otg_clk;
	u32 phypwr;
	int err;

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}

	/* set to suspend HSIC 0 and 1 and standard of PHY1 */
	phypwr = readl(EXYNOS4_PHYPWR);
	if (soc_is_exynos4210()) {
		phypwr |= (PHY1_STD_FORCE_SUSPEND
			| EXYNOS4210_HSIC0_FORCE_SUSPEND
			| EXYNOS4210_HSIC1_FORCE_SUSPEND);
	} else {
		phypwr = readl(EXYNOS4_PHYPWR);
		phypwr |= (PHY1_STD_FORCE_SUSPEND
			| EXYNOS4212_HSIC0_FORCE_SUSPEND
			| EXYNOS4212_HSIC1_FORCE_SUSPEND);
	}
	writel(phypwr, EXYNOS4_PHYPWR);

	clk_disable(otg_clk);
	clk_put(otg_clk);

	return 0;
}

static int exynos4_usb_phy1_resume(struct platform_device *pdev)
{
	struct clk *otg_clk;
	u32 rstcon;
	u32 phypwr;
	int err = 0;

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}

	if (exynos4_usb_host_phy_is_on()) {
		/* set to resume HSIC 0 and 1 and standard of PHY1 */
		phypwr = readl(EXYNOS4_PHYPWR);
		if (soc_is_exynos4210()) {
			phypwr &= ~(PHY1_STD_FORCE_SUSPEND
				| EXYNOS4210_HSIC0_FORCE_SUSPEND
				| EXYNOS4210_HSIC1_FORCE_SUSPEND);
		} else {
			/* USB MUX change from Device to Host */
			exynos4_usb_mux_change(1);

			phypwr = readl(EXYNOS4_PHYPWR);
			phypwr &= ~(PHY1_STD_FORCE_SUSPEND
				| EXYNOS4212_HSIC0_FORCE_SUSPEND
				| EXYNOS4212_HSIC1_FORCE_SUSPEND);
		}
		writel(phypwr, EXYNOS4_PHYPWR);
		err = 0;
	} else {
		phypwr = readl(EXYNOS4_PHYPWR);
		/* set to normal HSIC 0 and 1 of PHY1 */
		if (soc_is_exynos4210()) {
			exynos4_usb_phy_control(USB_PHY1, PHY_ENABLE);

			phypwr &= ~(PHY1_STD_NORMAL_MASK
				| EXYNOS4210_HSIC0_NORMAL_MASK
				| EXYNOS4210_HSIC1_NORMAL_MASK);
			writel(phypwr, EXYNOS4_PHYPWR);

			/* reset all ports of both PHY and Link */
			rstcon = readl(EXYNOS4_RSTCON)
				| EXYNOS4210_HOST_LINK_PORT_SWRST_MASK
				| EXYNOS4210_PHY1_SWRST_MASK;
			writel(rstcon, EXYNOS4_RSTCON);
			udelay(10);

			rstcon &= ~(EXYNOS4210_HOST_LINK_PORT_SWRST_MASK
				| EXYNOS4210_PHY1_SWRST_MASK);
			writel(rstcon, EXYNOS4_RSTCON);
		} else {
			exynos4_usb_phy_control(USB_PHY
				| USB_PHY_HSIC0
				| USB_PHY_HSIC1,
				PHY_ENABLE);

			/* USB MUX change from Device to Host */
			if (!exynos4_usb_device_phy_is_on())
				exynos4_usb_mux_change(1);

			phypwr &= ~(PHY1_STD_NORMAL_MASK
				| EXYNOS4212_HSIC0_NORMAL_MASK
				| EXYNOS4212_HSIC1_NORMAL_MASK);
			writel(phypwr, EXYNOS4_PHYPWR);

			/* reset all ports of both PHY and Link */
			rstcon = readl(EXYNOS4_RSTCON)
				| EXYNOS4212_HOST_LINK_PORT_SWRST_MASK
				| EXYNOS4212_PHY1_SWRST_MASK;
			writel(rstcon, EXYNOS4_RSTCON);
			udelay(10);

			rstcon &= ~(EXYNOS4212_HOST_LINK_PORT_SWRST_MASK
				| EXYNOS4212_PHY1_SWRST_MASK);
			writel(rstcon, EXYNOS4_RSTCON);
		}
		err = 1;
	}
	udelay(80);

	clk_disable(otg_clk);
	clk_put(otg_clk);

	return err;
}

static int exynos4_usb_phy1_init(struct platform_device *pdev)
{
	struct clk *otg_clk;
	u32 phypwr;
	u32 phyclk;
	u32 rstcon;
	int err;

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}

	if (soc_is_exynos4210()) {
		exynos4_usb_phy_control(USB_PHY1, PHY_ENABLE);
	} else {
		exynos4_usb_phy_control(USB_PHY
			| USB_PHY_HSIC0
			| USB_PHY_HSIC1,
			PHY_ENABLE);
	}

	atomic_inc(&host_usage);

	if (exynos4_usb_host_phy_is_on()) {
		dev_err(&pdev->dev, "Already power on PHY\n");
		clk_disable(otg_clk);
		clk_put(otg_clk);
		return 0;
	}

	/*
	 *  set XuhostOVERCUR to in-active by controlling ET6PUD[15:14]
	 *  0x0 : pull-up/down disabled
	 *  0x1 : pull-down enabled
	 *  0x2 : reserved
	 *  0x3 : pull-up enabled
	 */
	writel((__raw_readl(ETC6PUD) & ~(0x3 << 14)) | (0x3 << 14),
		ETC6PUD);

	/* set clock frequency for PLL */
	phyclk = exynos4_usb_phy_set_clock(pdev);
	writel(phyclk, EXYNOS4_PHYCLK);

	phypwr = readl(EXYNOS4_PHYPWR);
	/* set to normal HSIC 0 and 1 of PHY1 */
	if (soc_is_exynos4210()) {
		phypwr &= ~(PHY1_STD_NORMAL_MASK
			| EXYNOS4210_HSIC0_NORMAL_MASK
			| EXYNOS4210_HSIC1_NORMAL_MASK);
		writel(phypwr, EXYNOS4_PHYPWR);

		/* floating prevention logic: disable */
		writel((readl(EXYNOS4_PHY1CON) | FPENABLEN), EXYNOS4_PHY1CON);

		/* reset all ports of both PHY and Link */
		rstcon = readl(EXYNOS4_RSTCON)
			| EXYNOS4210_HOST_LINK_PORT_SWRST_MASK
			| EXYNOS4210_PHY1_SWRST_MASK;
		writel(rstcon, EXYNOS4_RSTCON);
		udelay(10);

		rstcon &= ~(EXYNOS4210_HOST_LINK_PORT_SWRST_MASK
			| EXYNOS4210_PHY1_SWRST_MASK);
		writel(rstcon, EXYNOS4_RSTCON);
	} else {
		/* USB MUX change from Device to Host */
		exynos4_usb_mux_change(1);

		phypwr &= ~(PHY1_STD_NORMAL_MASK
			| EXYNOS4212_HSIC0_NORMAL_MASK
			| EXYNOS4212_HSIC1_NORMAL_MASK);
		writel(phypwr, EXYNOS4_PHYPWR);

		/* reset all ports of both PHY and Link */
		rstcon = readl(EXYNOS4_RSTCON)
			| EXYNOS4212_HOST_LINK_PORT_SWRST_MASK
			| EXYNOS4212_PHY1_SWRST_MASK;
		writel(rstcon, EXYNOS4_RSTCON);
		udelay(10);

		rstcon &= ~(EXYNOS4212_HOST_LINK_PORT_SWRST_MASK
			| EXYNOS4212_PHY1_SWRST_MASK);
		writel(rstcon, EXYNOS4_RSTCON);
	}
	udelay(80);

	clk_disable(otg_clk);
	clk_put(otg_clk);

	return 0;
}

static int exynos4_usb_phy1_exit(struct platform_device *pdev)
{
	struct clk *otg_clk;
	u32 phypwr;
	int err;

	if (atomic_dec_return(&host_usage) > 0) {
		dev_info(&pdev->dev, "still being used\n");
		return -EBUSY;
	}

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}

	if (soc_is_exynos4210()) {
		phypwr = readl(EXYNOS4_PHYPWR)
			| PHY1_STD_NORMAL_MASK
			| EXYNOS4210_HSIC0_NORMAL_MASK
			| EXYNOS4210_HSIC1_NORMAL_MASK;
	} else {
		phypwr = readl(EXYNOS4_PHYPWR)
			| PHY1_STD_NORMAL_MASK
			| EXYNOS4212_HSIC0_NORMAL_MASK
			| EXYNOS4212_HSIC1_NORMAL_MASK;
	}
	writel(phypwr, EXYNOS4_PHYPWR);

	if (soc_is_exynos4210()) {
		exynos4_usb_phy_control(USB_PHY1, PHY_DISABLE);
	} else {
		exynos4_usb_phy_control(USB_PHY
			| USB_PHY_HSIC0
			| USB_PHY_HSIC1,
			PHY_DISABLE);
	}

	clk_disable(otg_clk);
	clk_put(otg_clk);

	return 0;
}

static int exynos5_usb_phy20_init(struct platform_device *pdev)
{
	struct clk *ext_xtal;
	u32 hostphy_ctrl0;
	u32 hsic_ctrl;
	u32 ehcictrl;

	hostphy_ctrl0 = readl(EXYNOS5_PHY_HOST_CTRL0);

	exynos4_usb_phy_control(USB_PHY1, PHY_ENABLE);

	if (exynos5_usb_host_phy20_is_on()) {
		dev_err(&pdev->dev, "Already power on PHY\n");
		return 0;
	}

	hostphy_ctrl0 &= ~(HOST_CTRL0_FSEL_MASK);

	ext_xtal = clk_get(&pdev->dev, "ext_xtal");
	switch (clk_get_rate(ext_xtal)) {
	case 96 * 100000:
		hostphy_ctrl0 |= EXYNOS5_CLKSEL_9600K;
		break;
	case 10 * MHZ:
		hostphy_ctrl0 |= EXYNOS5_CLKSEL_10M;
		break;
	case 12 * MHZ:
		hostphy_ctrl0 |= EXYNOS5_CLKSEL_12M;
		break;
	case 192 * 100000:
		hostphy_ctrl0 |= EXYNOS5_CLKSEL_19200K;
		break;
	case 20 * MHZ:
		hostphy_ctrl0 |= EXYNOS5_CLKSEL_20M;
		break;
	case 24 * MHZ:
		/* default reference clock */
		hostphy_ctrl0 |= EXYNOS5_CLKSEL_24M;
		break;
	default:
	case 50 * MHZ:
		/* default reference clock */
		hostphy_ctrl0 |= EXYNOS5_CLKSEL_50M;
		break;
	}

	hostphy_ctrl0 &= ~(HOST_CTRL0_COMMONONN);

	hostphy_ctrl0 &= ~(HOST_CTRL0_PHYSWRST | HOST_CTRL0_PHYSWRSTALL | HOST_CTRL0_SIDDQ);
	hostphy_ctrl0 &= ~(HOST_CTRL0_FORCESUSPEND | HOST_CTRL0_FORCESLEEP);
	hostphy_ctrl0 |= (HOST_CTRL0_LINKSWRST | HOST_CTRL0_UTMISWRST);
	writel(hostphy_ctrl0, EXYNOS5_PHY_HOST_CTRL0);
	hsic_ctrl = (HSIC_CTRL_REFCLKDIV(0x24) | HSIC_CTRL_REFCLKSEL(0x2) |
		HSIC_CTRL_PHYSWRST);
	writel(hsic_ctrl, EXYNOS5_PHY_HSIC_CTRL1);
	writel(hsic_ctrl, EXYNOS5_PHY_HSIC_CTRL2);
	udelay(1000);

	hsic_ctrl = (HSIC_CTRL_REFCLKDIV(0x24) | HSIC_CTRL_REFCLKSEL(0x2));
	writel(hsic_ctrl, EXYNOS5_PHY_HSIC_CTRL1);
	writel(hsic_ctrl, EXYNOS5_PHY_HSIC_CTRL2);
	hostphy_ctrl0 &= ~(HOST_CTRL0_LINKSWRST | HOST_CTRL0_UTMISWRST);
	writel(hostphy_ctrl0, EXYNOS5_PHY_HOST_CTRL0);

	ehcictrl = readl(EXYNOS5_PHY_HOST_EHCICTRL);
	ehcictrl |= (EHCICTRL_ENAINCRXALIGN | EHCICTRL_ENAINCR4 |
			EHCICTRL_ENAINCR8 | EHCICTRL_ENAINCR16);
	writel(ehcictrl, EXYNOS5_PHY_HOST_EHCICTRL);

	return 0;
}

static int exynos5_usb_phy20_exit(struct platform_device *pdev)
{
	u32 hostphy_ctrl0;
	u32 hsic_ctrl;

	hsic_ctrl = (HSIC_CTRL_REFCLKDIV(0x24) | HSIC_CTRL_REFCLKSEL(0x2) |
			HSIC_CTRL_SIDDQ | HSIC_CTRL_FORCESLEEP | HSIC_CTRL_FORCESUSPEND);
	writel(hsic_ctrl, EXYNOS5_PHY_HSIC_CTRL1);
	writel(hsic_ctrl, EXYNOS5_PHY_HSIC_CTRL2);

	hostphy_ctrl0 = readl(EXYNOS5_PHY_HOST_CTRL0);
	hostphy_ctrl0 |= (HOST_CTRL0_SIDDQ);
	hostphy_ctrl0 |= (HOST_CTRL0_FORCESUSPEND | HOST_CTRL0_FORCESLEEP);
	hostphy_ctrl0 |= (HOST_CTRL0_PHYSWRST | HOST_CTRL0_PHYSWRSTALL);
	writel(hostphy_ctrl0, EXYNOS5_PHY_HOST_CTRL0);

	exynos4_usb_phy_control(USB_PHY1, PHY_DISABLE);
	return 0;
}

static int exynos5_usb_phy30_init(struct platform_device *pdev)
{
	u32 reg;

	exynos4_usb_phy_control(USB_PHY0, PHY_ENABLE);

	/* Reset USB 3.0 PHY */
	writel(0x087fffc0, EXYNOS_USB3_LINKSYSTEM);
	writel(0x00000000, EXYNOS_USB3_PHYREG0);
	writel(0x24d4e6e4, EXYNOS_USB3_PHYPARAM0);
	writel(0x03fff815, EXYNOS_USB3_PHYPARAM1);
	writel(0x00000000, EXYNOS_USB3_PHYBATCHG);
	writel(0x00000000, EXYNOS_USB3_PHYRESUME);
	/* REVISIT : Over-current pin is inactive on SMDK5250 */
	if (soc_is_exynos5250())
		writel((readl(EXYNOS_USB3_LINKPORT) & ~(0x3<<4)) |
			(0x3<<2), EXYNOS_USB3_LINKPORT);

	/* UTMI Power Control */
	writel(EXYNOS_USB3_PHYUTMI_OTGDISABLE, EXYNOS_USB3_PHYUTMI);

	/* Set 100MHz external clock */
	reg = EXYNOS_USB3_PHYCLKRST_PORTRESET |
		/* HS PLL uses ref_pad_clk{p,m} or ref_alt_clk_{p,m}
		* as reference */
		EXYNOS_USB3_PHYCLKRST_REFCLKSEL(2) |
		/* Digital power supply in normal operating mode */
		EXYNOS_USB3_PHYCLKRST_RETENABLEN |
		/* 0x27-100MHz, 0x2a-24MHz, 0x31-20MHz, 0x38-19.2MHz */
		EXYNOS_USB3_PHYCLKRST_FSEL(0x27) |
		/* 0x19-100MHz, 0x68-24MHz, 0x7d-20Mhz */
		EXYNOS_USB3_PHYCLKRST_MPLL_MULTIPLIER(0x19) |
		/* Enable ref clock for SS function */
		EXYNOS_USB3_PHYCLKRST_REF_SSP_EN |
		/* Enable spread spectrum */
		EXYNOS_USB3_PHYCLKRST_SSC_EN;

	writel(reg, EXYNOS_USB3_PHYCLKRST);

	udelay(10);

	reg &= ~(EXYNOS_USB3_PHYCLKRST_PORTRESET);
	writel(reg, EXYNOS_USB3_PHYCLKRST);

	return 0;
}

static int exynos5_usb_phy30_exit(struct platform_device *pdev)
{
	u32 reg;

	reg = EXYNOS_USB3_PHYUTMI_OTGDISABLE |
		EXYNOS_USB3_PHYUTMI_FORCESUSPEND |
		EXYNOS_USB3_PHYUTMI_FORCESLEEP;
	writel(reg, EXYNOS_USB3_PHYUTMI);

	exynos4_usb_phy_control(USB_PHY0, PHY_DISABLE);

	return 0;
}

int s5p_usb_phy_suspend(struct platform_device *pdev, int type)
{
	if (type == S5P_USB_PHY_HOST)
		return exynos4_usb_phy1_suspend(pdev);

	return -EINVAL;
}

int s5p_usb_phy_resume(struct platform_device *pdev, int type)
{
	if (type == S5P_USB_PHY_HOST)
		return exynos4_usb_phy1_resume(pdev);

	return -EINVAL;
}

int s5p_usb_phy_init(struct platform_device *pdev, int type)
{
	if (type == S5P_USB_PHY_HOST) {
		if (soc_is_exynos4210() || soc_is_exynos4210() || soc_is_exynos4210())
			return exynos4_usb_phy1_init(pdev);
		else
			return exynos5_usb_phy20_init(pdev);
	} else if (type == S5P_USB_PHY_DEVICE) {
		return exynos4_usb_phy0_init(pdev);
	} else if (type == S5P_USB_PHY_DRD) {
		return exynos5_usb_phy30_init(pdev);
	}

	return -EINVAL;
}

int s5p_usb_phy_exit(struct platform_device *pdev, int type)
{
	if (type == S5P_USB_PHY_HOST) {
		if (soc_is_exynos4210() || soc_is_exynos4210() || soc_is_exynos4210())
			return exynos4_usb_phy1_exit(pdev);
		else
			return exynos5_usb_phy20_exit(pdev);
	} else if (type == S5P_USB_PHY_DEVICE) {
		return exynos4_usb_phy0_exit(pdev);
	} else if (type == S5P_USB_PHY_DRD) {
		return exynos5_usb_phy30_exit(pdev);
	}

	return -EINVAL;
}