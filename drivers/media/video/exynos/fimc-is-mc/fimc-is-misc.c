/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is misc functions(mipi, fimc-lite control)
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/memory.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>
#include <linux/jiffies.h>

#include <media/v4l2-subdev.h>
#include <media/exynos_fimc_is.h>
#include <linux/videodev2_samsung.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <plat/gpio-cfg.h>
#include <mach/map.h>
#include <mach/regs-clock.h>

#include "fimc-is-core.h"
#include "fimc-is-helper.h"

/* PMU for FIMC-IS*/
#define FIMCLITE0_REG_BASE		(S5P_VA_FIMCLITE0)  /* phy : 0x13c0_0000 */
#define FIMCLITE1_REG_BASE		(S5P_VA_FIMCLITE1)  /* phy : 0x13c1_0000 */
#define MIPICSI0_REG_BASE		(S5P_VA_MIPICSI0)   /* phy : 0x13c2_0000 */
#define MIPICSI1_REG_BASE		(S5P_VA_MIPICSI1)   /* phy : 0x13c3_0000 */

#define FLITE_MAX_RESET_READY_TIME	(20) /* 100ms */
#define FLITE_MAX_WIDTH_SIZE		(8192)
#define FLITE_MAX_HEIGHT_SIZE		(8192)


/*FIMCLite*/
/* Camera Source size */
#define FLITE_REG_CISRCSIZE				0x00
#define FLITE_REG_CISRCSIZE_SIZE_H(x)			((x) << 16)
#define FLITE_REG_CISRCSIZE_SIZE_V(x)			((x) << 0)
#define FLITE_REG_CISRCSIZE_ORDER422_IN_YCBYCR		(0 << 14)
#define FLITE_REG_CISRCSIZE_ORDER422_IN_YCRYCB		(1 << 14)
#define FLITE_REG_CISRCSIZE_ORDER422_IN_CBYCRY		(2 << 14)
#define FLITE_REG_CISRCSIZE_ORDER422_IN_CRYCBY		(3 << 14)

/* Global control */
#define FLITE_REG_CIGCTRL				0x04
#define FLITE_REG_CIGCTRL_YUV422_1P			(0x1E << 24)
#define FLITE_REG_CIGCTRL_RAW8				(0x2A << 24)
#define FLITE_REG_CIGCTRL_RAW10				(0x2B << 24)
#define FLITE_REG_CIGCTRL_RAW12				(0x2C << 24)
#define FLITE_REG_CIGCTRL_RAW14				(0x2D << 24)

/* User defined formats. x = 0...0xF. */
#define FLITE_REG_CIGCTRL_USER(x)			(0x30 + x - 1)
#define FLITE_REG_CIGCTRL_SHADOWMASK_DISABLE		(1 << 21)
#define FLITE_REG_CIGCTRL_ODMA_DISABLE			(1 << 20)
#define FLITE_REG_CIGCTRL_SWRST_REQ			(1 << 19)
#define FLITE_REG_CIGCTRL_SWRST_RDY			(1 << 18)
#define FLITE_REG_CIGCTRL_SWRST				(1 << 17)
#define FLITE_REG_CIGCTRL_TEST_PATTERN_COLORBAR		(1 << 15)
#define FLITE_REG_CIGCTRL_INVPOLPCLK			(1 << 14)
#define FLITE_REG_CIGCTRL_INVPOLVSYNC			(1 << 13)
#define FLITE_REG_CIGCTRL_INVPOLHREF			(1 << 12)
#define FLITE_REG_CIGCTRL_IRQ_LASTEN0_ENABLE		(0 << 8)
#define FLITE_REG_CIGCTRL_IRQ_LASTEN0_DISABLE		(1 << 8)
#define FLITE_REG_CIGCTRL_IRQ_ENDEN0_ENABLE		(0 << 7)
#define FLITE_REG_CIGCTRL_IRQ_ENDEN0_DISABLE		(1 << 7)
#define FLITE_REG_CIGCTRL_IRQ_STARTEN0_ENABLE		(0 << 6)
#define FLITE_REG_CIGCTRL_IRQ_STARTEN0_DISABLE		(1 << 6)
#define FLITE_REG_CIGCTRL_IRQ_OVFEN0_ENABLE		(0 << 5)
#define FLITE_REG_CIGCTRL_IRQ_OVFEN0_DISABLE		(1 << 5)
#define FLITE_REG_CIGCTRL_SELCAM_MIPI			(1 << 3)

/* Image Capture Enable */
#define FLITE_REG_CIIMGCPT				0x08
#define FLITE_REG_CIIMGCPT_IMGCPTEN			(1 << 31)
#define FLITE_REG_CIIMGCPT_CPT_FREN			(1 << 25)
#define FLITE_REG_CIIMGCPT_CPT_FRPTR(x)			((x) << 19)
#define FLITE_REG_CIIMGCPT_CPT_MOD_FRCNT		(1 << 18)
#define FLITE_REG_CIIMGCPT_CPT_MOD_FREN			(0 << 18)
#define FLITE_REG_CIIMGCPT_CPT_FRCNT(x)			((x) << 10)

/* Capture Sequence */
#define FLITE_REG_CICPTSEQ				0x0C
#define FLITE_REG_CPT_FRSEQ(x)				((x) << 0)

/* Camera Window Offset */
#define FLITE_REG_CIWDOFST				0x10
#define FLITE_REG_CIWDOFST_WINOFSEN			(1 << 31)
#define FLITE_REG_CIWDOFST_CLROVIY			(1 << 31)
#define FLITE_REG_CIWDOFST_WINHOROFST(x)		((x) << 16)
#define FLITE_REG_CIWDOFST_HOROFF_MASK			(0x1fff << 16)
#define FLITE_REG_CIWDOFST_CLROVFICB			(1 << 15)
#define FLITE_REG_CIWDOFST_CLROVFICR			(1 << 14)
#define FLITE_REG_CIWDOFST_WINVEROFST(x)		((x) << 0)
#define FLITE_REG_CIWDOFST_VEROFF_MASK			(0x1fff << 0)

/* Cmaera Window Offset2 */
#define FLITE_REG_CIWDOFST2				0x14
#define FLITE_REG_CIWDOFST2_WINHOROFST2(x)		((x) << 16)
#define FLITE_REG_CIWDOFST2_WINVEROFST2(x)		((x) << 0)

/* Camera Output DMA Format */
#define FLITE_REG_CIODMAFMT				0x18
#define FLITE_REG_CIODMAFMT_1D_DMA			(1 << 15)
#define FLITE_REG_CIODMAFMT_2D_DMA			(0 << 15)
#define FLITE_REG_CIODMAFMT_PACK12			(1 << 14)
#define FLITE_REG_CIODMAFMT_NORMAL			(0 << 14)
#define FLITE_REG_CIODMAFMT_CRYCBY			(0 << 4)
#define FLITE_REG_CIODMAFMT_CBYCRY			(1 << 4)
#define FLITE_REG_CIODMAFMT_YCRYCB			(2 << 4)
#define FLITE_REG_CIODMAFMT_YCBYCR			(3 << 4)

/* Camera Output Canvas */
#define FLITE_REG_CIOCAN				0x20
#define FLITE_REG_CIOCAN_OCAN_V(x)			((x) << 16)
#define FLITE_REG_CIOCAN_OCAN_H(x)			((x) << 0)

/* Camera Output DMA Offset */
#define FLITE_REG_CIOOFF				0x24
#define FLITE_REG_CIOOFF_OOFF_V(x)			((x) << 16)
#define FLITE_REG_CIOOFF_OOFF_H(x)			((x) << 0)

/* Camera Output DMA Address */
#define FLITE_REG_CIOSA					0x30
#define FLITE_REG_CIOSA_OSA(x)				((x) << 0)

/* Camera Status */
#define FLITE_REG_CISTATUS				0x40
#define FLITE_REG_CISTATUS_MIPI_VVALID			(1 << 22)
#define FLITE_REG_CISTATUS_MIPI_HVALID			(1 << 21)
#define FLITE_REG_CISTATUS_MIPI_DVALID			(1 << 20)
#define FLITE_REG_CISTATUS_ITU_VSYNC			(1 << 14)
#define FLITE_REG_CISTATUS_ITU_HREFF			(1 << 13)
#define FLITE_REG_CISTATUS_OVFIY			(1 << 10)
#define FLITE_REG_CISTATUS_OVFICB			(1 << 9)
#define FLITE_REG_CISTATUS_OVFICR			(1 << 8)
#define FLITE_REG_CISTATUS_IRQ_SRC_OVERFLOW		(1 << 7)
#define FLITE_REG_CISTATUS_IRQ_SRC_LASTCAPEND		(1 << 6)
#define FLITE_REG_CISTATUS_IRQ_SRC_FRMSTART		(1 << 5)
#define FLITE_REG_CISTATUS_IRQ_SRC_FRMEND		(1 << 4)
#define FLITE_REG_CISTATUS_IRQ_CAM			(1 << 0)
#define FLITE_REG_CISTATUS_IRQ_MASK			(0xf << 4)

/* Camera Status2 */
#define FLITE_REG_CISTATUS2				0x44
#define FLITE_REG_CISTATUS2_LASTCAPEND			(1 << 1)
#define FLITE_REG_CISTATUS2_FRMEND			(1 << 0)

/* Qos Threshold */
#define FLITE_REG_CITHOLD				0xF0
#define FLITE_REG_CITHOLD_W_QOS_EN			(1 << 30)
#define FLITE_REG_CITHOLD_WTH_QOS(x)			((x) << 0)

/* Camera General Purpose */
#define FLITE_REG_CIGENERAL				0xFC
#define FLITE_REG_CIGENERAL_CAM_A			(0 << 0)
#define FLITE_REG_CIGENERAL_CAM_B			(1 << 0)


/*MIPI*/
/* CSIS global control */
#define S5PCSIS_CTRL					0x00
#define S5PCSIS_CTRL_DPDN_DEFAULT			(0 << 31)
#define S5PCSIS_CTRL_DPDN_SWAP				(1 << 31)
#define S5PCSIS_CTRL_ALIGN_32BIT			(1 << 20)
#define S5PCSIS_CTRL_UPDATE_SHADOW			(1 << 16)
#define S5PCSIS_CTRL_WCLK_EXTCLK			(1 << 8)
#define S5PCSIS_CTRL_RESET				(1 << 4)
#define S5PCSIS_CTRL_ENABLE				(1 << 0)

/* D-PHY control */
#define S5PCSIS_DPHYCTRL				0x04
#define S5PCSIS_DPHYCTRL_HSS_MASK			(0x1f << 27)
#define S5PCSIS_DPHYCTRL_ENABLE				(0x7 << 0)

#define S5PCSIS_CONFIG					0x08
#define S5PCSIS_CFG_FMT_YCBCR422_8BIT			(0x1e << 2)
#define S5PCSIS_CFG_FMT_RAW8				(0x2a << 2)
#define S5PCSIS_CFG_FMT_RAW10				(0x2b << 2)
#define S5PCSIS_CFG_FMT_RAW12				(0x2c << 2)
/* User defined formats, x = 1...4 */
#define S5PCSIS_CFG_FMT_USER(x)				((0x30 + x - 1) << 2)
#define S5PCSIS_CFG_FMT_MASK				(0x3f << 2)
#define S5PCSIS_CFG_NR_LANE_MASK			3

/* Interrupt mask. */
#define S5PCSIS_INTMSK					0x10
#define S5PCSIS_INTMSK_EN_ALL				0xf000103f
#define S5PCSIS_INTSRC					0x14

/* Pixel resolution */
#define S5PCSIS_RESOL					0x2c
#define CSIS_MAX_PIX_WIDTH				0xffff
#define CSIS_MAX_PIX_HEIGHT				0xffff

static void flite_hw_set_cam_source_size(unsigned long flite_reg_base,
											struct flite_frame *f_frame)
{
	u32 cfg = 0;

	cfg = readl(flite_reg_base + FLITE_REG_CISRCSIZE);

	cfg |= FLITE_REG_CISRCSIZE_SIZE_H(f_frame->o_width);
	cfg |= FLITE_REG_CISRCSIZE_SIZE_V(f_frame->o_height);

	writel(cfg, flite_reg_base + FLITE_REG_CISRCSIZE);

	cfg = readl(flite_reg_base + FLITE_REG_CIOCAN);
	cfg |= FLITE_REG_CIOCAN_OCAN_H(f_frame->o_width);
	cfg |= FLITE_REG_CIOCAN_OCAN_V(f_frame->o_height);

	writel(cfg, flite_reg_base + FLITE_REG_CIOCAN);
}

static void flite_hw_set_cam_channel(unsigned long flite_reg_base)
{
	u32 cfg = readl(flite_reg_base + FLITE_REG_CIGENERAL);


	if (flite_reg_base == (unsigned long)FIMCLITE0_REG_BASE)
		cfg &= FLITE_REG_CIGENERAL_CAM_A;
	else
		cfg &= FLITE_REG_CIGENERAL_CAM_B;

	writel(cfg, FIMCLITE0_REG_BASE + FLITE_REG_CIGENERAL);

}

static void flite_hw_set_capture_start(unsigned long flite_reg_base)
{
	u32 cfg = 0;

	cfg = readl(flite_reg_base + FLITE_REG_CIIMGCPT);
	cfg |= FLITE_REG_CIIMGCPT_IMGCPTEN;

	writel(cfg, flite_reg_base + FLITE_REG_CIIMGCPT);
}

static void flite_hw_set_capture_stop(unsigned long flite_reg_base)
{
	u32 cfg = 0;

	cfg = readl(flite_reg_base + FLITE_REG_CIIMGCPT);
	cfg &= ~FLITE_REG_CIIMGCPT_IMGCPTEN;

	writel(cfg, flite_reg_base + FLITE_REG_CIIMGCPT);
}

static int flite_hw_set_source_format(unsigned long flite_reg_base)
{
	u32 cfg = 0;

	cfg = readl(flite_reg_base + FLITE_REG_CIGCTRL);
	cfg |= FLITE_REG_CIGCTRL_RAW10;
	writel(cfg, flite_reg_base + FLITE_REG_CIGCTRL);

	return 0;
}

static void flite_hw_set_output_dma(unsigned long flite_reg_base,
										bool enable)
{
	u32 cfg = 0;
	cfg = readl(flite_reg_base + FLITE_REG_CIGCTRL);

	if (enable)
		cfg &= ~FLITE_REG_CIGCTRL_ODMA_DISABLE;
	else
		cfg |= FLITE_REG_CIGCTRL_ODMA_DISABLE;

	writel(cfg, flite_reg_base + FLITE_REG_CIGCTRL);
}

/* will use for pattern generation testing
static void flite_hw_set_test_pattern_enable(void)
{
	u32 cfg = 0;
	cfg = readl(flite_reg_base + FLITE_REG_CIGCTRL);
	cfg |= FLITE_REG_CIGCTRL_TEST_PATTERN_COLORBAR;

	writel(cfg, flite_reg_base + FLITE_REG_CIGCTRL);
}
*/

static void flite_hw_set_config_irq(unsigned long flite_reg_base)
{
	u32 cfg = 0;
	cfg = readl(flite_reg_base + FLITE_REG_CIGCTRL);
	cfg &= ~(FLITE_REG_CIGCTRL_INVPOLPCLK | FLITE_REG_CIGCTRL_INVPOLVSYNC
			| FLITE_REG_CIGCTRL_INVPOLHREF);

	writel(cfg, flite_reg_base + FLITE_REG_CIGCTRL);
}

static void flite_hw_set_interrupt_source(unsigned long flite_reg_base)
{
	u32 cfg = 0;
	cfg = readl(flite_reg_base + FLITE_REG_CIGCTRL);
	cfg |= FLITE_REG_CIGCTRL_IRQ_LASTEN0_ENABLE;

	writel(cfg, flite_reg_base + FLITE_REG_CIGCTRL);
}

static void flite_hw_set_interrupt_starten0_disable(unsigned long flite_reg_base)
{
	u32 cfg = 0;
	cfg = readl(flite_reg_base + FLITE_REG_CIGCTRL);
	cfg |= FLITE_REG_CIGCTRL_IRQ_STARTEN0_DISABLE;

	writel(cfg, flite_reg_base + FLITE_REG_CIGCTRL);
}

static void flite_hw_set_camera_type(unsigned long flite_reg_base)
{
	u32 cfg = 0;
	cfg = readl(flite_reg_base + FLITE_REG_CIGCTRL);

	cfg |= FLITE_REG_CIGCTRL_SELCAM_MIPI;

	writel(cfg, flite_reg_base + FLITE_REG_CIGCTRL);
}

static void flite_hw_set_window_offset(unsigned long flite_reg_base,
											struct flite_frame *f_frame)
{
	u32 cfg = 0;
	u32 hoff2, voff2;

	cfg = readl(flite_reg_base + FLITE_REG_CIWDOFST);
	cfg &= ~(FLITE_REG_CIWDOFST_HOROFF_MASK |
		FLITE_REG_CIWDOFST_VEROFF_MASK);
	cfg |= FLITE_REG_CIWDOFST_WINOFSEN |
		FLITE_REG_CIWDOFST_WINHOROFST(f_frame->offs_h) |
		FLITE_REG_CIWDOFST_WINVEROFST(f_frame->offs_v);

	writel(cfg, flite_reg_base + FLITE_REG_CIWDOFST);

	hoff2 = f_frame->o_width - f_frame->width - f_frame->offs_h;
	voff2 = f_frame->o_height - f_frame->height - f_frame->offs_v;
	cfg = FLITE_REG_CIWDOFST2_WINHOROFST2(hoff2) |
		FLITE_REG_CIWDOFST2_WINVEROFST2(voff2);

	writel(cfg, flite_reg_base + FLITE_REG_CIWDOFST2);
}

static void flite_hw_set_last_capture_end_clear(unsigned long flite_reg_base)
{
	u32 cfg = 0;

	cfg = readl(flite_reg_base + FLITE_REG_CISTATUS2);
	cfg &= ~FLITE_REG_CISTATUS2_LASTCAPEND;

	writel(cfg, flite_reg_base + FLITE_REG_CISTATUS2);
}

static void s5pcsis_enable_interrupts(unsigned long mipi_reg_base, bool on)
{
	u32 val = readl(mipi_reg_base + S5PCSIS_INTMSK);

	val = on ? val | S5PCSIS_INTMSK_EN_ALL :
		   val & ~S5PCSIS_INTMSK_EN_ALL;
	writel(val, mipi_reg_base + S5PCSIS_INTMSK);
}

static void s5pcsis_reset(unsigned long mipi_reg_base)
{
	u32 val = readl(mipi_reg_base + S5PCSIS_CTRL);

	writel(val | S5PCSIS_CTRL_RESET, mipi_reg_base + S5PCSIS_CTRL);
	udelay(10);
}

static void s5pcsis_system_enable(unsigned long mipi_reg_base, int on)
{
	u32 val;

	val = readl(mipi_reg_base + S5PCSIS_CTRL);
	if (on) {
		val |= S5PCSIS_CTRL_ENABLE;
		val |= S5PCSIS_CTRL_WCLK_EXTCLK;
	} else
		val &= ~S5PCSIS_CTRL_ENABLE;
	writel(val, mipi_reg_base + S5PCSIS_CTRL);

	val = readl(mipi_reg_base + S5PCSIS_DPHYCTRL);
	if (on)
		val |= S5PCSIS_DPHYCTRL_ENABLE;
	else
		val &= ~S5PCSIS_DPHYCTRL_ENABLE;
	writel(val, mipi_reg_base + S5PCSIS_DPHYCTRL);
}

/* Called with the state.lock mutex held */
static void __s5pcsis_set_format(unsigned long mipi_reg_base, struct flite_frame *f_frame)
{
	u32 val;

	/* Color format */
	val = readl(mipi_reg_base + S5PCSIS_CONFIG);
	val = (val & ~S5PCSIS_CFG_FMT_MASK) | S5PCSIS_CFG_FMT_RAW10;
	writel(val, mipi_reg_base + S5PCSIS_CONFIG);

	/* Pixel resolution */
	val = (f_frame->o_width << 16) | f_frame->o_height;
	writel(val, mipi_reg_base + S5PCSIS_RESOL);
}

static void s5pcsis_set_hsync_settle(unsigned long mipi_reg_base)
{
	u32 val = readl(mipi_reg_base + S5PCSIS_DPHYCTRL);

	val = (val & ~S5PCSIS_DPHYCTRL_HSS_MASK) | (0x6 << 28);
	writel(val, mipi_reg_base + S5PCSIS_DPHYCTRL);
}

static void s5pcsis_set_params(unsigned long mipi_reg_base, struct flite_frame *f_frame)
{
	u32 val;

	val = readl(mipi_reg_base + S5PCSIS_CONFIG);
	val = (val & ~S5PCSIS_CFG_NR_LANE_MASK) | (2 - 1);
	writel(val, mipi_reg_base + S5PCSIS_CONFIG);

	__s5pcsis_set_format(mipi_reg_base, f_frame);
	s5pcsis_set_hsync_settle(mipi_reg_base);

	val = readl(mipi_reg_base + S5PCSIS_CTRL);
	val &= ~S5PCSIS_CTRL_ALIGN_32BIT;

	/* Not using external clock. */
	val &= ~S5PCSIS_CTRL_WCLK_EXTCLK;

	writel(val, mipi_reg_base + S5PCSIS_CTRL);

	/* Update the shadow register. */
	val = readl(mipi_reg_base + S5PCSIS_CTRL);
	writel(val | S5PCSIS_CTRL_UPDATE_SHADOW, mipi_reg_base + S5PCSIS_CTRL);
}

int start_fimc_lite(int channel, struct flite_frame *f_frame)
{
	unsigned long base_reg = (unsigned long)FIMCLITE0_REG_BASE;

	if (channel == FLITE_ID_A)
		base_reg = (unsigned long)FIMCLITE0_REG_BASE;
	else if (channel == FLITE_ID_B)
		base_reg = (unsigned long)FIMCLITE1_REG_BASE;

	flite_hw_set_cam_channel(base_reg);
	flite_hw_set_cam_source_size(base_reg, f_frame);
	flite_hw_set_camera_type(base_reg);
	flite_hw_set_source_format(base_reg);
	flite_hw_set_output_dma(base_reg, false);

	flite_hw_set_interrupt_source(base_reg);
	flite_hw_set_interrupt_starten0_disable(base_reg);
	flite_hw_set_config_irq(base_reg);
	flite_hw_set_window_offset(base_reg, f_frame);
	/* flite_hw_set_test_pattern_enable(); */

	flite_hw_set_last_capture_end_clear(base_reg);
	flite_hw_set_capture_start(base_reg);

	return 0;
}

int stop_fimc_lite(int channel)
{
	unsigned long base_reg = (unsigned long)FIMCLITE0_REG_BASE;

	if (channel == FLITE_ID_A)
		base_reg = (unsigned long)FIMCLITE0_REG_BASE;
	else if (channel == FLITE_ID_B)
		base_reg = (unsigned long)FIMCLITE1_REG_BASE;

	flite_hw_set_capture_stop(base_reg);
	return 0;
}

int enable_mipi(void)
{
	void __iomem *addr;
	u32 cfg;

	addr = S5P_MIPI_DPHY_CONTROL(0);

	cfg = __raw_readl(addr);
	cfg = (cfg | S5P_MIPI_DPHY_SRESETN);
	__raw_writel(cfg, addr);

	if (1) {
		cfg |= S5P_MIPI_DPHY_ENABLE;
	} else if (!(cfg & (S5P_MIPI_DPHY_SRESETN |
				S5P_MIPI_DPHY_MRESETN) & (~S5P_MIPI_DPHY_SRESETN))) {
		cfg &= ~S5P_MIPI_DPHY_ENABLE;
	}

	__raw_writel(cfg, addr);


	addr = S5P_MIPI_DPHY_CONTROL(1);

	cfg = __raw_readl(addr);
	cfg = (cfg | S5P_MIPI_DPHY_SRESETN);
	__raw_writel(cfg, addr);

	if (1) {
		cfg |= S5P_MIPI_DPHY_ENABLE;
	} else if (!(cfg & (S5P_MIPI_DPHY_SRESETN |
				S5P_MIPI_DPHY_MRESETN) & (~S5P_MIPI_DPHY_SRESETN))) {
		cfg &= ~S5P_MIPI_DPHY_ENABLE;
	}

	__raw_writel(cfg, addr);
	return 0;

}

int start_mipi_csi(int channel, struct flite_frame *f_frame)
{
	unsigned long base_reg = (unsigned long)MIPICSI0_REG_BASE;

	if (channel == CSI_ID_A)
		base_reg = (unsigned long)MIPICSI0_REG_BASE;
	else if (channel == CSI_ID_B)
		base_reg = (unsigned long)MIPICSI1_REG_BASE;

	s5pcsis_reset(base_reg);
	s5pcsis_set_params(base_reg, f_frame);
	s5pcsis_system_enable(base_reg, true);
	s5pcsis_enable_interrupts(base_reg, true);

	return 0;
}

int stop_mipi_csi(int channel)
{
	unsigned long base_reg = (unsigned long)MIPICSI0_REG_BASE;

	if (channel == CSI_ID_A)
		base_reg = (unsigned long)MIPICSI0_REG_BASE;
	else if (channel == CSI_ID_B)
		base_reg = (unsigned long)MIPICSI1_REG_BASE;

	s5pcsis_enable_interrupts(base_reg, false);
	s5pcsis_system_enable(base_reg, false);

	return 0;
}

/*
* will be move to setting file
*/
int fimc_is_v4l2_af_start_stop(struct fimc_is_dev *dev, int value)
{
	int ret = 0;
	switch (value) {
#if 0 /* will be implemented*/
	case AUTO_FOCUS_OFF:
		if (!is_af_use(dev)) {
			/* 6A3 can't support AF */
			dev->af.af_state = FIMC_IS_AF_IDLE;
		} else {
			if (dev->af.af_state == FIMC_IS_AF_IDLE)
				return ret;
			/* Abort or lock AF */
			dev->af.af_state = FIMC_IS_AF_ABORT;
			IS_ISP_SET_PARAM_AA_CMD(dev, ISP_AA_COMMAND_STOP);
			IS_ISP_SET_PARAM_AA_TARGET(dev, ISP_AA_TARGET_AF);
			switch (dev->af.mode) {
			case IS_FOCUS_MODE_AUTO:
				IS_ISP_SET_PARAM_AA_MODE(dev,
					ISP_AF_MODE_SINGLE);
				IS_ISP_SET_PARAM_AA_SCENE(dev,
					ISP_AF_SCENE_NORMAL);
				IS_ISP_SET_PARAM_AA_SLEEP(dev,
					ISP_AF_SLEEP_OFF);
				IS_ISP_SET_PARAM_AA_FACE(dev,
					ISP_AF_FACE_DISABLE);
				IS_ISP_SET_PARAM_AA_TOUCH_X(dev, 0);
				IS_ISP_SET_PARAM_AA_TOUCH_Y(dev, 0);
				IS_ISP_SET_PARAM_AA_MANUAL_AF(dev, 0);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
					break;
			case IS_FOCUS_MODE_MACRO:
				IS_ISP_SET_PARAM_AA_MODE(dev,
					ISP_AF_MODE_SINGLE);
				IS_ISP_SET_PARAM_AA_SCENE(dev,
					ISP_AF_SCENE_MACRO);
				IS_ISP_SET_PARAM_AA_SLEEP(dev,
					ISP_AF_SLEEP_OFF);
				IS_ISP_SET_PARAM_AA_FACE(dev,
					ISP_AF_FACE_DISABLE);
				IS_ISP_SET_PARAM_AA_TOUCH_X(dev, 0);
				IS_ISP_SET_PARAM_AA_TOUCH_Y(dev, 0);
				IS_ISP_SET_PARAM_AA_MANUAL_AF(dev, 0);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE), HZ/5);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
				break;
			case IS_FOCUS_MODE_CONTINUOUS:
				IS_ISP_SET_PARAM_AA_MODE(dev,
					ISP_AF_MODE_CONTINUOUS);
				IS_ISP_SET_PARAM_AA_SCENE(dev,
					ISP_AF_SCENE_NORMAL);
				IS_ISP_SET_PARAM_AA_SLEEP(dev,
						ISP_AF_SLEEP_OFF);
				IS_ISP_SET_PARAM_AA_FACE(dev,
					ISP_AF_FACE_DISABLE);
				IS_ISP_SET_PARAM_AA_TOUCH_X(dev, 0);
				IS_ISP_SET_PARAM_AA_TOUCH_Y(dev, 0);
				IS_ISP_SET_PARAM_AA_MANUAL_AF(dev, 0);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE), HZ/5);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
				break;
			default:
				/* If other AF mode, there is no
				cancelation process*/
				break;
			}
			dev->af.mode = IS_FOCUS_MODE_IDLE;
		}
		break;
#endif
	case AUTO_FOCUS_ON:
		if (!is_af_use(dev)) {
			printk("6A3 doesn't use AF\n");
			/* 6A3 can't support AF */
			dev->af.af_state = FIMC_IS_AF_LOCK;
			dev->af.af_lock_state = FIMC_IS_AF_LOCKED;
		}
		else{
			printk("auto focus start\n");
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_SINGLE);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean(
				(void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			ret = wait_event_timeout(dev->irq_queue,
			(dev->af.af_state == FIMC_IS_AF_RUNNING), HZ/5);
			if (!ret) {
				dev_err(&dev->pdev->dev,
				"Focus change timeout:%s\n", __func__);
				return -EBUSY;
			}
			printk("auto focus end\n");
		}
		break;

	default:
		break;
	}
	return ret;
}


