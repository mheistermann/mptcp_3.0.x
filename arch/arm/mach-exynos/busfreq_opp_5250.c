/* linux/arch/arm/mach-exynos/busfreq_opp_5250.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS5 - BUS clock frequency scaling support with OPP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/ktime.h>
#include <linux/tick.h>
#include <linux/kernel_stat.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/opp.h>
#include <linux/clk.h>
#include <mach/busfreq_exynos5.h>

#include <asm/mach-types.h>

#include <mach/ppmu.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/gpio.h>
#include <mach/regs-mem.h>
#include <mach/dev.h>
#include <mach/asv.h>
#include <mach/regs-pmu-5250.h>

#include <plat/map-s5p.h>
#include <plat/gpio-cfg.h>
#include <plat/cpu.h>
#include <plat/clock.h>

#include <mach/asv-exynos.h>

#define MIF_MAX_THRESHOLD	13
#define INT_MAX_THRESHOLD	8
#define MIF_IDLE_THRESHOLD	3
#define INT_IDLE_THRESHOLD	2

#ifdef CONFIG_S5P_DP
#define MIF_LOCK_LCD		300160
#else
#define MIF_LOCK_LCD		300133
#endif

#define INT_RBB		6	/* +300mV */

static bool init_done;

static const unsigned int max_threshold[PPMU_TYPE_END] = {
	MIF_MAX_THRESHOLD,
	INT_MAX_THRESHOLD,
};

static const unsigned int idle_threshold[PPMU_TYPE_END] = {
	MIF_IDLE_THRESHOLD,
	INT_IDLE_THRESHOLD,
};

static struct device busfreq_for_int;

static struct busfreq_table exynos5_busfreq_table_for800[] = {
	{LV_0, 800000, 1000000, 0, 0, 0},
	{LV_1, 667000, 1000000, 0, 0, 0},
	{LV_2, 400000, 1000000, 0, 0, 0},
	{LV_3, 267000, 1000000, 0, 0, 0},
	{LV_4, 160000, 1000000, 0, 0, 0},
	{LV_5, 100000, 1000000, 0, 0, 0},
};

static struct busfreq_table exynos5_busfreq_table_for667[] = {
	{LV_0, 667000, 1000000, 0, 0, 0},
	{LV_1, 334000, 1000000, 0, 0, 0},
	{LV_2, 111000, 1000000, 0, 0, 0},
	{LV_3, 111000, 1000000, 0, 0, 0},
	{LV_4, 111000, 1000000, 0, 0, 0},
	{LV_5, 111000, 1000000, 0, 0, 0},
};

static struct busfreq_table exynos5_busfreq_table_for533[] = {
	{LV_0, 533000, 1000000, 0, 0, 0},
	{LV_1, 267000, 1000000, 0, 0, 0},
	{LV_2, 107000, 1000000, 0, 0, 0},
	{LV_3, 107000, 1000000, 0, 0, 0},
	{LV_4, 107000, 1000000, 0, 0, 0},
	{LV_5, 107000, 1000000, 0, 0, 0},
};

static struct busfreq_table exynos5_busfreq_table_for400[] = {
	{LV_0, 400000, 1000000, 0, 0, 0},
	{LV_1, 267000, 1000000, 0, 0, 0},
	{LV_2, 100000, 1000000, 0, 0, 0},
	{LV_3, 100000, 1000000, 0, 0, 0},
	{LV_4, 100000, 1000000, 0, 0, 0},
	{LV_5, 100000, 1000000, 0, 0, 0},
};
#define ASV_GROUP_10	10
#define ASV_GROUP_12	12

#define MIF_ASV_GROUP	3

static unsigned int asv_group_index;
static unsigned int mif_asv_group_index;

static unsigned int exynos5_mif_volt_for800_orig[ASV_GROUP_10+1][LV_MIF_END] = {
	/* L0        L1       L2       L3      L4      L5 */
	{      0,       0,       0,       0,       0,      0}, /* ASV0 */
	{1200000, 1100000, 1000000, 1000000,  950000, 950000}, /* ASV1 */
	{1200000, 1050000, 1000000,  950000,  950000, 900000}, /* ASV2 */
	{1200000, 1050000, 1050000, 1000000,  950000, 950000}, /* ASV3 */
	{1150000, 1050000, 1000000,  950000,  950000, 900000}, /* ASV4 */
	{1150000, 1050000, 1050000, 1000000, 1000000, 950000}, /* ASV5 */
	{1150000, 1050000, 1050000,  950000,  950000, 950000}, /* ASV6 */
	{1100000, 1050000, 1000000,  950000,  900000, 900000}, /* ASV7 */
	{1100000, 1050000, 1000000,  950000,  900000, 900000}, /* ASV8 */
	{1100000, 1050000, 1000000,  950000,  900000, 900000}, /* ASV9 */
	{1100000, 1050000, 1000000,  950000,  900000, 900000}, /* ASV10 */
};

static unsigned int exynos5_mif_volt_for800[MIF_ASV_GROUP+1][LV_MIF_END] = {
	/* L0        L1       L2       L3       L4       L5 */
	{1150000, 1050000, 1050000, 1000000, 1000000, 1000000}, /* ASV0 */
	{1100000, 1000000,  950000,  950000,  950000,  900000}, /* ASV1 */
	{1050000,  950000,  950000,  950000,  900000,  900000}, /* ASV2 */
	{1000000,  900000,  900000,  900000,  900000,  900000}, /* ASV3 */
};

static unsigned int exynos5_mif_volt_for667[ASV_GROUP_10+1][LV_MIF_END] = {
	/* L0        L1       L2       L3       L4       L5 */
	{      0,       0,       0,       0,       0,       0}, /* ASV0 */
	{1100000, 1000000,  950000,  950000,  950000,  950000}, /* ASV1 */
	{1050000, 1000000,  950000,  950000,  950000,  950000}, /* ASV2 */
	{1050000, 1050000,  950000,  950000,  950000,  950000}, /* ASV3 */
	{1050000, 1000000,  950000,  950000,  950000,  950000}, /* ASV4 */
	{1050000, 1050000, 1000000, 1000000, 1000000, 1000000}, /* ASV5 */
	{1050000, 1050000,  950000,  950000,  950000,  950000}, /* ASV6 */
	{1050000, 1000000,  900000,  900000,  900000,  900000}, /* ASV7 */
	{1050000, 1000000,  900000,  900000,  900000,  900000}, /* ASV8 */
	{1050000, 1000000,  900000,  900000,  900000,  900000}, /* ASV9 */
	{1050000, 1000000,  900000,  900000,  900000,  900000}, /* ASV10 */
};

static unsigned int exynos5_mif_volt_for533[ASV_GROUP_10+1][LV_MIF_END] = {
	/* L0        L1       L2       L3        L4      L5 */
	{      0,       0,       0,       0,       0,       0}, /* ASV0 */
	{1050000, 1000000,  950000,  950000,  950000,  950000}, /* ASV1 */
	{1000000,  950000,  950000,  950000,  950000,  950000}, /* ASV2 */
	{1050000, 1000000,  950000,  950000,  950000,  950000}, /* ASV3 */
	{1000000,  950000,  950000,  950000,  950000,  950000}, /* ASV4 */
	{1050000, 1000000, 1000000, 1000000, 1000000, 1000000}, /* ASV5 */
	{1050000,  950000,  950000,  950000,  950000,  950000}, /* ASV6 */
	{1000000,  950000,  900000,  900000,  900000,  900000}, /* ASV7 */
	{1000000,  950000,  900000,  900000,  900000,  900000}, /* ASV8 */
	{1000000,  950000,  900000,  900000,  900000,  900000}, /* ASV9 */
	{1000000,  950000,  900000,  900000,  900000,  900000}, /* ASV10 */
};

static unsigned int exynos5_mif_volt_for400[ASV_GROUP_10+1][LV_MIF_END] = {
	/* L0        L1      L2      L3      L4      L5 */
	{      0,       0,      0,      0,      0,      0}, /* ASV0 */
	{1000000, 1000000, 950000, 950000, 950000, 950000}, /* ASV1 */
	{1000000,  950000, 900000, 900000, 900000, 900000}, /* ASV2 */
	{1050000, 1000000, 950000, 950000, 950000, 950000}, /* ASV3 */
	{1000000,  950000, 900000, 900000, 900000, 900000}, /* ASV4 */
	{1050000, 1000000, 950000, 950000, 950000, 950000}, /* ASV5 */
	{1050000,  950000, 950000, 950000, 950000, 950000}, /* ASV6 */
	{1000000,  950000, 900000, 900000, 900000, 900000}, /* ASV7 */
	{1000000,  950000, 900000, 900000, 900000, 900000}, /* ASV8 */
	{1000000,  950000, 900000, 900000, 900000, 900000}, /* ASV9 */
	{1000000,  950000, 900000, 900000, 900000, 900000}, /* ASV10 */
};

static struct busfreq_table *exynos5_busfreq_table_mif;

static unsigned int (*exynos5_mif_volt)[LV_MIF_END];

static struct busfreq_table exynos5_busfreq_table_int[] = {
	{LV_0, 267000, 1000000, 0, 0, 0},
	{LV_1, 200000, 1000000, 0, 0, 0},
	{LV_2, 160000, 1000000, 0, 0, 0},
	{LV_3, 133000, 1000000, 0, 0, 0},
};

static unsigned int exynos5_int_volt[ASV_GROUP_12][LV_INT_END] = {
	/* L0        L1       L2       L3 */
	{1037500, 1000000, 975000, 950000}, /* ASV0 */
	{1025000,  975000, 962500, 937500}, /* ASV1 */
	{1025000,  987500, 975000, 950000}, /* ASV2 */
	{1012500,  975000, 962500, 937500}, /* ASV3 */
	{1000000,  962500, 950000, 925000}, /* ASV4 */
	{ 987500,  950000, 937500, 912500}, /* ASV5 */
	{ 975000,  937500, 925000, 900000}, /* ASV6 */
	{ 962500,  925000, 912500, 900000}, /* ASV7 */
	{ 962500,  925000, 912500, 900000}, /* ASV8 */
	{ 950000,  912500, 900000, 900000}, /* ASV9 */
	{ 950000,  912500, 900000, 900000}, /* ASV10 */
	{ 937500,  900000, 900000, 887500}, /* ASV11 */
};

/* For CMU_LEX */
static unsigned int clkdiv_lex[LV_INT_END][2] = {
	/*
	 * Clock divider value for following
	 * { DIVATCLK_LEX, DIVPCLK_LEX }
	 */

	/* ATCLK_LEX L0 : 200MHz */
	{0, 1},

	/* ATCLK_LEX L1 : 166MHz */
	{0, 1},

	/* ATCLK_LEX L2 : 133MHz */
	{0, 1},

	/* ATCLK_LEX L3 : 114MHz */
	{0, 1},
};

/* For CMU_R0X */
static unsigned int clkdiv_r0x[LV_INT_END][1] = {
	/*
	 * Clock divider value for following
	 * { DIVPCLK_R0X }
	 */

	/* ACLK_PR0X L0 : 133MHz */
	{1},

	/* ACLK_DR0X L1 : 100MHz */
	{1},

	/* ACLK_PR0X L2 : 80MHz */
	{1},

	/* ACLK_PR0X L3 : 67MHz */
	{1},
};

/* For CMU_R1X */
static unsigned int clkdiv_r1x[LV_INT_END][1] = {
	/*
	 * Clock divider value for following
	 * { DIVPCLK_R1X }
	 */

	/* ACLK_PR1X L0 : 133MHz */
	{1},

	/* ACLK_DR1X L1 : 100MHz */
	{1},

	/* ACLK_PR1X L2 : 80MHz */
	{1},

	/* ACLK_PR1X L3 : 67MHz */
	{1},
};

/* For CMU_TOP */
static unsigned int clkdiv_top[LV_INT_END][10] = {
	/*
	 * Clock divider value for following
	 * { DIVACLK400_ISP, DIVACLK400_IOP, DIVACLK266, DIVACLK_200, DIVACLK_66_PRE,
	 DIVACLK_66, DIVACLK_333, DIVACLK_166, DIVACLK_300_DISP1, DIVACLK300_GSCL }
	 */

	/* ACLK_400_ISP L0 : 400MHz */
	{1, 1, 2, 3, 1, 5, 0, 1, 0, 0},

	/* ACLK_400_ISP L1 : 267MHz */
	{2, 3, 3, 4, 1, 5, 1, 2, 0, 0},

	/* ACLK_400_ISP L2 : 200MHz */
	{3, 3, 4, 4, 1, 5, 2, 3, 0, 0},

	/* ACLK_400_ISP L3 : 160MHz */
	{4, 4, 5, 5, 1, 5, 2, 3, 1, 1},
};

/* For CMU_ACP */
static unsigned int clkdiv_acp[LV_MIF_END][2] = {
	/*
	 * Clock divider value for following
	 * { DIVACLK_SYSLFT, DIVPCLK_SYSLFT }
	 */

	/* ACLK_SYSLFT L0 : 400MHz */
	{1, 1},

	/* ACLK_SYSLFT L1 : 400MHz */
	{1, 1},

	/* ACLK_SYSLFT L2 : 200MHz */
	{3, 1},

	/* ACLK_SYSLFT L3 : 133MHz */
	{5, 1},

	/* ACLK_SYSLFT L4 : 100MHz */
	{7, 1},

	/* ACLK_SYSLFT L5 : 100MHz */
	{7, 1},
};

/* For CMU_CORE */
static unsigned int clkdiv_core[LV_MIF_END][1] = {
	/*
	 * Clock divider value for following
	 * { DIVACLK_R1BX }
	 */

	/* ACLK_SYSLFT L0 : 400MHz */
	{1},

	/* ACLK_SYSLFT L1 : 400MHz */
	{1},

	/* ACLK_SYSLFT L2 : 200MHz */
	{3},

	/* ACLK_SYSLFT L3 : 133MHz */
	{5},

	/* ACLK_SYSLFT L4 : 100MHz */
	{7},

	/* ACLK_SYSLFT L5 : 100MHz */
	{7},
};

/* For CMU_CDREX */
static unsigned int __maybe_unused clkdiv_cdrex_for800[LV_MIF_END][9] = {
	/*
	 * Clock divider value for following
	 * { DIVMCLK_DPHY, DIVMCLK_CDREX2, DIVACLK_CDREX, DIVMCLK_CDREX,
		DIVPCLK_CDREX, DIVC2C, DIVC2C_ACLK, DIVMCLK_EFPHY, DIVACLK_EFCON }
	 */

	/* MCLK_CDREX L0: 800MHz */
	{0, 0, 1, 2, 1, 1, 1, 4, 1},

	/* MCLK_CDREX L1: 667MHz */
	{0, 0, 1, 2, 1, 1, 1, 4, 1},

	/* MCLK_CDREX L2: 400MHz */
	{0, 1, 1, 2, 3, 2, 1, 5, 1},

	/* MCLK_CDREX L3: 267MHz */
	{0, 2, 1, 2, 4, 2, 1, 5, 1},

	/* MCLK_CDREX L4: 160MHz */
	{0, 4, 1, 2, 5, 2, 1, 5, 1},

	/* MCLK_CDREX L5: 100MHz */
	{0, 7, 1, 2, 6, 7, 1, 15, 1},
};

static unsigned int __maybe_unused clkdiv_cdrex_for667[LV_MIF_END][9] = {
	/*
	 * Clock divider value for following
	 * { DIVMCLK_DPHY, DIVMCLK_CDREX2, DIVACLK_CDREX, DIVMCLK_CDREX,
		DIVPCLK_CDREX, DIVC2C, DIVC2C_ACLK, DIVMCLK_EFPHY, DIVACLK_EFCON }
	 */

	/* MCLK_CDREX L0: 667MHz */
	{0, 0, 1, 0, 4, 1, 1, 4, 1},

	/* MCLK_CDREX L1: 334MHz */
	{0, 1, 1, 1, 4, 2, 1, 5, 1},

	/* MCLK_CDREX L2: 111MHz */
	{0, 5, 1, 4, 4, 5, 1, 8, 1},

	/* MCLK_CDREX L3: 111MHz */
	{0, 5, 1, 4, 4, 5, 1, 8, 1},

	/* MCLK_CDREX L4: 111MHz */
	{0, 5, 1, 4, 4, 5, 1, 8, 1},

	/* MCLK_CDREX L5: 111MHz */
	{0, 5, 1, 4, 4, 5, 1, 8, 1},
};

static unsigned int clkdiv_cdrex_for533[LV_MIF_END][9] = {
	/*
	 * Clock divider value for following
	 * { DIVMCLK_DPHY, DIVMCLK_CDREX2, DIVACLK_CDREX, DIVMCLK_CDREX,
		DIVPCLK_CDREX, DIVC2C, DIVC2C_ACLK, DIVMCLK_EFPHY, DIVACLK_EFCON }
	 */

	/* MCLK_CDREX L0: 533MHz */
	{0, 0, 1, 0, 3, 1, 1, 4, 1},

	/* MCLK_CDREX L1: 267MHz */
	{0, 1, 1, 1, 3, 2, 1, 5, 1},

	/* MCLK_CDREX L2: 107MHz */
	{0, 4, 1, 4, 3, 5, 1, 8, 1},

	/* MCLK_CDREX L3: 107MHz */
	{0, 4, 1, 4, 3, 5, 1, 8, 1},

	/* MCLK_CDREX L4: 107MHz */
	{0, 4, 1, 4, 3, 5, 1, 8, 1},

	/* MCLK_CDREX L5: 107MHz */
	{0, 4, 1, 4, 3, 5, 1, 8, 1},
};

static unsigned int __maybe_unused clkdiv_cdrex_for400[LV_MIF_END][9] = {
	/*
	 * Clock divider value for following
	 * { DIVMCLK_DPHY, DIVMCLK_CDREX2, DIVACLK_CDREX, DIVMCLK_CDREX,
		DIVPCLK_CDREX, DIVC2C, DIVC2C_ACLK, DIVMCLK_EFPHY, DIVACLK_EFCON }
	 */

	/* MCLK_CDREX L0: 400MHz */
	{1, 1, 1, 0, 5, 1, 1, 4, 1},

	/* MCLK_CDREX L1: 267MHz */
	{1, 2, 1, 2, 2, 2, 1, 5, 1},

	/* MCLK_CDREX L2: 100MHz */
	{1, 7, 1, 2, 7, 7, 1, 15, 1},

	/* MCLK_CDREX L3: 100MHz */
	{1, 7, 1, 2, 7, 7, 1, 15, 1},

	/* MCLK_CDREX L4: 100MHz */
	{1, 7, 1, 2, 7, 7, 1, 15, 1},

	/* MCLK_CDREX L5: 100MHz */
	{1, 7, 1, 2, 7, 7, 1, 15, 1},
};

static unsigned int (*clkdiv_cdrex)[9];

static void exynos5250_set_bus_volt(void)
{
	unsigned int i;

	if (soc_is_exynos5250() && samsung_rev() < EXYNOS5250_REV_1_0) {
		asv_group_index = 0;
		mif_asv_group_index = 0;
	} else {
		asv_group_index = exynos_result_of_asv;
		mif_asv_group_index = exynos_result_mif_asv;
	}

	if (asv_group_index == 0xff) {
		asv_group_index = 0;
		mif_asv_group_index = 0;
	}

	for (i = LV_0; i < LV_MIF_END; i++)
		exynos5_busfreq_table_mif[i].volt =
			get_match_volt(ID_MIF, exynos5_busfreq_table_mif[i].mem_clk);

	for (i = LV_0; i < LV_INT_END; i++) {
		exynos5_busfreq_table_int[i].volt =
			get_match_volt(ID_INT, exynos5_busfreq_table_int[i].mem_clk);
	}

	printk(KERN_INFO "DVFS : VDD_INT Voltage table set with %d Group\n", asv_group_index);
	printk(KERN_INFO "DVFS : VDD_INT Voltage of L0 level is %d \n", exynos5_busfreq_table_int[LV_0].volt);
	printk(KERN_INFO "DVFS : VDD_MIF Voltage table set with %d Group\n", mif_asv_group_index);
	printk(KERN_INFO "DVFS : VDD_MIF Voltage of L0 level is %d \n",	exynos5_busfreq_table_mif[LV_0].volt);

	return;
}

unsigned int old_mif_index = 0;

static struct clk *mout_bpll;
static struct clk *mout_mpll;
static struct clk *mout_mclk_cdrex;

static void exynos5250_mif_div_change(struct busfreq_data *data, int div_index)
{
	unsigned int tmp;

	/* Change Divier - CORE */
	tmp = __raw_readl(EXYNOS5_CLKDIV_SYSRGT);
	tmp &= ~EXYNOS5_CLKDIV_SYSRGT_ACLK_R1BX_MASK;

	tmp |= clkdiv_core[div_index][0];

	__raw_writel(tmp, EXYNOS5_CLKDIV_SYSLFT);

	/* Change Divider - CDREX */
	tmp = data->cdrex_divtable[div_index];

	__raw_writel(tmp, EXYNOS5_CLKDIV_CDREX);

	if (samsung_rev() < EXYNOS5250_REV_1_0) {
		tmp = data->cdrex2_divtable[div_index];

		__raw_writel(tmp, EXYNOS5_CLKDIV_CDREX2);
	}

	/* Change Divier - ACP */
	tmp = __raw_readl(EXYNOS5_CLKDIV_SYSLFT);

	tmp &= ~(EXYNOS5_CLKDIV_SYSLFT_PCLK_SYSLFT_MASK |
		 EXYNOS5_CLKDIV_SYSLFT_ACLK_SYSLFT_MASK);

	tmp |= ((clkdiv_acp[div_index][0] << EXYNOS5_CLKDIV_SYSLFT_ACLK_SYSLFT_SHIFT) |
		(clkdiv_acp[div_index][1] << EXYNOS5_CLKDIV_SYSLFT_PCLK_SYSLFT_SHIFT));

	__raw_writel(tmp, EXYNOS5_CLKDIV_SYSLFT);
}

static void exynos5250_target_for_mif(struct busfreq_data *data, int div_index)
{
	/* Mux change BPLL to MPLL */
	if (old_mif_index == LV_1) {
		/* Change divider */
		exynos5250_mif_div_change(data, div_index);
		/* Change Mux BPLL to MPLL */
		if (clk_set_parent(mout_mclk_cdrex, mout_mpll))
			printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
					mout_mpll->name, mout_mclk_cdrex->name);
	/* Mux change MPLL to BPLL */
	} else if (div_index == LV_1) {
		/* Change Mux MPLL to BPLL */
		if (clk_set_parent(mout_mclk_cdrex, mout_bpll))
			printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
					mout_bpll->name, mout_mclk_cdrex->name);
		/* Change divider */
		exynos5250_mif_div_change(data, div_index);
	/* It is not need to change mux */
	} else
		/* Change divider */
		exynos5250_mif_div_change(data, div_index);

	old_mif_index = div_index;
}

static void exynos5250_target_for_int(struct busfreq_data *data, int div_index)
{
	unsigned int tmp;
	unsigned int tmp2;

	/* Change Divider - TOP */
	tmp = __raw_readl(EXYNOS5_CLKDIV_TOP0);

	tmp &= ~(EXYNOS5_CLKDIV_TOP0_ACLK266_MASK |
		EXYNOS5_CLKDIV_TOP0_ACLK200_MASK |
		EXYNOS5_CLKDIV_TOP0_ACLK66_MASK |
		EXYNOS5_CLKDIV_TOP0_ACLK333_MASK |
		EXYNOS5_CLKDIV_TOP0_ACLK166_MASK |
		EXYNOS5_CLKDIV_TOP0_ACLK300_DISP1_MASK);

	tmp |= ((clkdiv_top[div_index][2] << EXYNOS5_CLKDIV_TOP0_ACLK266_SHIFT) |
		(clkdiv_top[div_index][3] << EXYNOS5_CLKDIV_TOP0_ACLK200_SHIFT) |
		(clkdiv_top[div_index][5] << EXYNOS5_CLKDIV_TOP0_ACLK66_SHIFT) |
		(clkdiv_top[div_index][6] << EXYNOS5_CLKDIV_TOP0_ACLK333_SHIFT) |
		(clkdiv_top[div_index][7] << EXYNOS5_CLKDIV_TOP0_ACLK166_SHIFT) |
		(clkdiv_top[div_index][8] << EXYNOS5_CLKDIV_TOP0_ACLK300_DISP1_SHIFT));

	__raw_writel(tmp, EXYNOS5_CLKDIV_TOP0);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_TOP0);
	} while (tmp & 0x151101);

	tmp = __raw_readl(EXYNOS5_CLKDIV_TOP1);

	tmp &= ~(EXYNOS5_CLKDIV_TOP1_ACLK400_ISP_MASK |
		EXYNOS5_CLKDIV_TOP1_ACLK400_IOP_MASK |
		EXYNOS5_CLKDIV_TOP1_ACLK66_PRE_MASK |
		EXYNOS5_CLKDIV_TOP1_ACLK300_GSCL_MASK);

	tmp |= ((clkdiv_top[div_index][0] << EXYNOS5_CLKDIV_TOP1_ACLK400_ISP_SHIFT) |
		(clkdiv_top[div_index][1] << EXYNOS5_CLKDIV_TOP1_ACLK400_IOP_SHIFT) |
		(clkdiv_top[div_index][4] << EXYNOS5_CLKDIV_TOP1_ACLK66_PRE_SHIFT) |
		(clkdiv_top[div_index][9] << EXYNOS5_CLKDIV_TOP1_ACLK300_GSCL_SHIFT));


	__raw_writel(tmp, EXYNOS5_CLKDIV_TOP1);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_TOP1);
		tmp2 = __raw_readl(EXYNOS5_CLKDIV_STAT_TOP0);
	} while ((tmp & 0x1110000) && (tmp2 & 0x80000));

	/* Change Divider - LEX */
	tmp = data->lex_divtable[div_index];

	__raw_writel(tmp, EXYNOS5_CLKDIV_LEX);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_LEX);
	} while (tmp & 0x110);

	/* Change Divider - R0X */
	tmp = __raw_readl(EXYNOS5_CLKDIV_R0X);

	tmp &= ~EXYNOS5_CLKDIV_R0X_PCLK_R0X_MASK;

	tmp |= (clkdiv_r0x[div_index][0] << EXYNOS5_CLKDIV_R0X_PCLK_R0X_SHIFT);

	__raw_writel(tmp, EXYNOS5_CLKDIV_R0X);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_R0X);
	} while (tmp & 0x10);

	/* Change Divider - R1X */
	tmp = data->r1x_divtable[div_index];

	__raw_writel(tmp, EXYNOS5_CLKDIV_R1X);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_R1X);
	} while (tmp & 0x10);
}

static void exynos5250_target(struct busfreq_data *data, enum ppmu_type type,
			      int index)
{
	if (type == PPMU_MIF)
		exynos5250_target_for_mif(data, index);
	else
		exynos5250_target_for_int(data, index);
}

static int exynos5250_get_table_index(unsigned long freq, enum ppmu_type type)
{
	int index;

	if (type == PPMU_MIF) {
		for (index = LV_0; index < LV_MIF_END; index++)
			if (freq == exynos5_busfreq_table_mif[index].mem_clk)
				return index;
	} else {
		for (index = LV_0; index < LV_INT_END; index++)
			if (freq == exynos5_busfreq_table_int[index].mem_clk)
				return index;
	}
	return -EINVAL;
}


int exynos5250_find_busfreq_by_volt(unsigned int req_volt, unsigned int *freq)
{
	unsigned int volt_cmp;
	unsigned int tmp = 0;
	int i;

	/* check if req_volt has value or not */
	if (!init_done || !req_volt) {
		pr_err("%s: req_volt has no value.\n", __func__);
		return -EINVAL;
	}

	/* find busfreq level in busfreq_table */
	for (i = LV_MIF_END - 1; i >= 0; i--) {
		volt_cmp = exynos5_mif_volt[mif_asv_group_index][i];

		if (volt_cmp >= req_volt) {
			tmp = exynos5_busfreq_table_mif[i].mem_clk;
			break;
		}
	}

	if (tmp == 0)
		return -EINVAL;

	*freq = tmp;

	for (i = LV_INT_END - 1; i >= 0; i--) {
		volt_cmp = exynos5_int_volt[asv_group_index][i];

		if (volt_cmp >= req_volt) {
			tmp = exynos5_busfreq_table_int[i].mem_clk;
			break;
		}
	}

	if (tmp == 0) {
	pr_err("%s: %u volt can't support\n", __func__, req_volt);
		return -EINVAL;
	}

	*freq += (tmp / 1000);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos5250_find_busfreq_by_volt);

static void exynos5250_suspend(void)
{
	/* Nothing to do */
}

static void exynos5250_resume(void)
{
}

static void exynos5250_monitor(struct busfreq_data *data,
			struct opp **mif_opp, struct opp **int_opp)
{
	int i;
	unsigned long tmpfreq;
	unsigned long cpufreq = 0;
	unsigned long freq[PPMU_TYPE_END];
	unsigned long cpu_load;
	unsigned long ddr_c_load;
	unsigned long right0_load;
	unsigned long ddr_r1_load;
	unsigned long ddr_l_load;
	unsigned long load[PPMU_TYPE_END];
	unsigned int cpu_load_average = 0;
	unsigned int ddr_c_load_average = 0;
	unsigned int ddr_l_load_average = 0;
	unsigned int right0_load_average = 0;
	unsigned int ddr_r1_load_average = 0;
	unsigned int load_average[PPMU_TYPE_END];
	struct opp *opp[PPMU_TYPE_END];
	unsigned long lockfreq[PPMU_TYPE_END];
	unsigned long newfreq[PPMU_TYPE_END];

	ppmu_update(data->dev[PPMU_MIF], 3);

	/* Convert from base xxx to base maxfreq */
	cpu_load = div64_u64(ppmu_load[PPMU_CPU] * data->curr_freq[PPMU_MIF], data->max_freq[PPMU_MIF]);
	ddr_c_load = div64_u64(ppmu_load[PPMU_DDR_C] * data->curr_freq[PPMU_MIF], data->max_freq[PPMU_MIF]);
	right0_load = div64_u64(ppmu_load[PPMU_RIGHT0_BUS] * data->curr_freq[PPMU_INT], data->max_freq[PPMU_INT]);
	ddr_r1_load = div64_u64(ppmu_load[PPMU_DDR_R1] * data->curr_freq[PPMU_MIF], data->max_freq[PPMU_MIF]);
	ddr_l_load = div64_u64(ppmu_load[PPMU_DDR_L] * data->curr_freq[PPMU_MIF], data->max_freq[PPMU_MIF]);

	data->load_history[PPMU_CPU][data->index] = cpu_load;
	data->load_history[PPMU_DDR_C][data->index] = ddr_c_load;
	data->load_history[PPMU_RIGHT0_BUS][data->index] = right0_load;
	data->load_history[PPMU_DDR_R1][data->index] = ddr_r1_load;
	data->load_history[PPMU_DDR_L][data->index++] = ddr_l_load;

	if (data->index >= LOAD_HISTORY_SIZE)
		data->index = 0;

	for (i = 0; i < LOAD_HISTORY_SIZE; i++) {
		cpu_load_average += data->load_history[PPMU_CPU][i];
		ddr_c_load_average += data->load_history[PPMU_DDR_C][i];
		right0_load_average += data->load_history[PPMU_RIGHT0_BUS][i];
		ddr_r1_load_average += data->load_history[PPMU_DDR_R1][i];
		ddr_l_load_average += data->load_history[PPMU_DDR_L][i];
	}

	/* Calculate average Load */
	cpu_load_average /= LOAD_HISTORY_SIZE;
	ddr_c_load_average /= LOAD_HISTORY_SIZE;
	right0_load_average /= LOAD_HISTORY_SIZE;
	ddr_r1_load_average /= LOAD_HISTORY_SIZE;
	ddr_l_load_average /= LOAD_HISTORY_SIZE;

	if (ddr_c_load >= ddr_r1_load) {
		load[PPMU_MIF] = ddr_c_load;
		load_average[PPMU_MIF] = ddr_c_load_average;
	} else {
		load[PPMU_MIF] = ddr_r1_load;
		load_average[PPMU_MIF] = ddr_r1_load_average;
	}

	if (ddr_l_load >= load[PPMU_MIF]) {
		load[PPMU_MIF] = ddr_l_load;
		load_average[PPMU_MIF] = ddr_l_load_average;
	}

	load[PPMU_INT] = right0_load;
	load_average[PPMU_INT] = right0_load_average;

	for (i = PPMU_MIF; i < PPMU_TYPE_END; i++) {
		if (load[i] >= max_threshold[i]) {
			freq[i] = data->max_freq[i];
		} else if (load[i] < idle_threshold[i]) {
			if (load_average[i] < idle_threshold[i])
				freq[i] = step_down(data, i, 1);
			else
				freq[i] = data->curr_freq[i];
		} else {
			if (load[i] < load_average[i]) {
				load[i] = load_average[i];
				if (load[i] >= max_threshold[i])
					load[i] = max_threshold[i];
			}
			freq[i] = div64_u64(data->max_freq[i] * load[i], max_threshold[i]);
		}
	}

	tmpfreq = dev_max_freq(data->dev[PPMU_MIF]);
	lockfreq[PPMU_MIF] = (tmpfreq / 1000) * 1000;
	lockfreq[PPMU_INT] = (tmpfreq % 1000) * 1000;

	newfreq[PPMU_MIF] = max3(lockfreq[PPMU_MIF], freq[PPMU_MIF], cpufreq);
	newfreq[PPMU_INT] = max(lockfreq[PPMU_INT], freq[PPMU_INT]);
	opp[PPMU_MIF] = opp_find_freq_ceil(data->dev[PPMU_MIF], &newfreq[PPMU_MIF]);
	opp[PPMU_INT] = opp_find_freq_ceil(data->dev[PPMU_INT], &newfreq[PPMU_INT]);

	*mif_opp = opp[PPMU_MIF];
	*int_opp = opp[PPMU_INT];
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void busfreq_early_suspend(struct early_suspend *h)
{
	unsigned long freq;
	struct busfreq_data *data = container_of(h, struct busfreq_data,
			busfreq_early_suspend_handler);
	freq = data->min_freq[PPMU_MIF] + data->min_freq[PPMU_INT] / 1000;
	dev_lock(data->dev[PPMU_MIF], data->dev[PPMU_MIF], freq);
}

static void busfreq_late_resume(struct early_suspend *h)
{
	struct busfreq_data *data = container_of(h, struct busfreq_data,
			busfreq_early_suspend_handler);
	/* Request min 300MHz */
	dev_lock(data->dev[PPMU_MIF], data->dev[PPMU_MIF], MIF_LOCK_LCD);
}
#endif

int exynos5250_init(struct device *dev, struct busfreq_data *data, bool pop)
{
	unsigned int i, tmp;
	unsigned long maxfreq = ULONG_MAX;
	unsigned long minfreq = 0;
	unsigned long cdrexfreq;
	unsigned long lrbusfreq;
	struct clk *clk;
	int ret;

	mout_mpll = clk_get(NULL, "mout_mpll");
	if (IS_ERR(mout_mpll)) {
		dev_err(dev, "Fail to get mout_mpll clock");
		ret = PTR_ERR(mout_mpll);
		return ret;
	}

	mout_bpll = clk_get(NULL, "mout_bpll");
	if (IS_ERR(mout_bpll)) {
		dev_err(dev, "fail to get mout_bpll clock");
		ret = PTR_ERR(mout_bpll);
		return ret;
	}

	mout_mclk_cdrex = clk_get(NULL, "mclk_cdrex");
	if (IS_ERR(mout_mclk_cdrex)) {
		dev_err(dev, "fail to get mout_mclk_cdrex clock");
		ret = PTR_ERR(mout_mclk_cdrex);
		return ret;
	}

	clk = clk_get(NULL, "mclk_cdrex");
	if (IS_ERR(clk)) {
		dev_err(dev, "Fail to get mclk_cdrex clock");
		ret = PTR_ERR(clk);
		return ret;
	}
	cdrexfreq = clk_get_rate(clk) / 1000;
	clk_put(clk);

	clk = clk_get(NULL, "aclk_266");
	if (IS_ERR(clk)) {
		dev_err(dev, "Fail to get aclk_266 clock");
		ret = PTR_ERR(clk);
		return ret;
	}
	lrbusfreq = clk_get_rate(clk) / 1000;
	clk_put(clk);

	if (cdrexfreq == 800000) {
		clkdiv_cdrex = clkdiv_cdrex_for800;
		exynos5_busfreq_table_mif = exynos5_busfreq_table_for800;
		if (!exynos_lot_id)
			exynos5_mif_volt = exynos5_mif_volt_for800;
		else
			exynos5_mif_volt = exynos5_mif_volt_for800_orig;
	} else if (cdrexfreq == 666857) {
		clkdiv_cdrex = clkdiv_cdrex_for667;
		exynos5_busfreq_table_mif = exynos5_busfreq_table_for667;
		exynos5_mif_volt = exynos5_mif_volt_for667;
	} else if (cdrexfreq == 533000) {
		clkdiv_cdrex = clkdiv_cdrex_for533;
		exynos5_busfreq_table_mif = exynos5_busfreq_table_for533;
		exynos5_mif_volt = exynos5_mif_volt_for533;
	} else if (cdrexfreq == 400000) {
		clkdiv_cdrex = clkdiv_cdrex_for400;
		exynos5_busfreq_table_mif = exynos5_busfreq_table_for400;
		exynos5_mif_volt = exynos5_mif_volt_for400;
	} else {
		dev_err(dev, "Don't support cdrex table\n");
		return -EINVAL;
	}

	tmp = __raw_readl(EXYNOS5_CLKDIV_LEX);

	for (i = LV_0; i < LV_INT_END; i++) {
		tmp &= ~(EXYNOS5_CLKDIV_LEX_ATCLK_LEX_MASK | EXYNOS5_CLKDIV_LEX_PCLK_LEX_MASK);

		tmp |= ((clkdiv_lex[i][0] << EXYNOS5_CLKDIV_LEX_ATCLK_LEX_SHIFT) |
			(clkdiv_lex[i][1] << EXYNOS5_CLKDIV_LEX_PCLK_LEX_SHIFT));

		data->lex_divtable[i] = tmp;
	}

	tmp = __raw_readl(EXYNOS5_CLKDIV_R0X);

	for (i = LV_0; i < LV_INT_END; i++) {

		tmp &= ~EXYNOS5_CLKDIV_R0X_PCLK_R0X_MASK;

		tmp |= (clkdiv_r0x[i][0] << EXYNOS5_CLKDIV_R0X_PCLK_R0X_SHIFT);

		data->r0x_divtable[i] = tmp;
	}

	tmp = __raw_readl(EXYNOS5_CLKDIV_R1X);

	for (i = LV_0; i < LV_INT_END; i++) {
		tmp &= ~EXYNOS5_CLKDIV_R1X_PCLK_R1X_MASK;

		tmp |= (clkdiv_r1x[i][0] << EXYNOS5_CLKDIV_R1X_PCLK_R1X_SHIFT);

		data->r1x_divtable[i] = tmp;
	}

	tmp = __raw_readl(EXYNOS5_CLKDIV_CDREX);

	for (i = LV_0; i < LV_MIF_END; i++) {
		tmp &= ~(EXYNOS5_CLKDIV_CDREX_MCLK_DPHY_MASK |
			 EXYNOS5_CLKDIV_CDREX_MCLK_CDREX2_MASK |
			 EXYNOS5_CLKDIV_CDREX_ACLK_CDREX_MASK |
			 EXYNOS5_CLKDIV_CDREX_MCLK_CDREX_MASK |
			 EXYNOS5_CLKDIV_CDREX_PCLK_CDREX_MASK |
			 EXYNOS5_CLKDIV_CDREX_ACLK_CLK400_MASK |
			 EXYNOS5_CLKDIV_CDREX_ACLK_C2C200_MASK |
			 EXYNOS5_CLKDIV_CDREX_ACLK_EFCON_MASK);

		tmp |= ((clkdiv_cdrex[i][0] << EXYNOS5_CLKDIV_CDREX_MCLK_DPHY_SHIFT) |
			(clkdiv_cdrex[i][1] << EXYNOS5_CLKDIV_CDREX_MCLK_CDREX2_SHIFT) |
			(clkdiv_cdrex[i][2] << EXYNOS5_CLKDIV_CDREX_ACLK_CDREX_SHIFT) |
			(clkdiv_cdrex[i][3] << EXYNOS5_CLKDIV_CDREX_MCLK_CDREX_SHIFT) |
			(clkdiv_cdrex[i][4] << EXYNOS5_CLKDIV_CDREX_PCLK_CDREX_SHIFT) |
			(clkdiv_cdrex[i][5] << EXYNOS5_CLKDIV_CDREX_ACLK_CLK400_SHIFT) |
			(clkdiv_cdrex[i][6] << EXYNOS5_CLKDIV_CDREX_ACLK_C2C200_SHIFT) |
			(clkdiv_cdrex[i][8] << EXYNOS5_CLKDIV_CDREX_ACLK_EFCON_SHIFT));

		data->cdrex_divtable[i] = tmp;
	}

	if (samsung_rev() < EXYNOS5250_REV_1_0) {
		tmp = __raw_readl(EXYNOS5_CLKDIV_CDREX2);

		for (i = LV_0; i < LV_MIF_END; i++) {
			tmp &= ~EXYNOS5_CLKDIV_CDREX2_MCLK_EFPHY_MASK;

			tmp |= clkdiv_cdrex[i][7] << EXYNOS5_CLKDIV_CDREX2_MCLK_EFPHY_SHIFT;

			data->cdrex2_divtable[i] = tmp;

		}
	}

	exynos5250_set_bus_volt();

	data->dev[PPMU_MIF] = dev;
	data->dev[PPMU_INT] = &busfreq_for_int;

	for (i = LV_0; i < LV_MIF_END; i++) {
		ret = opp_add(data->dev[PPMU_MIF], exynos5_busfreq_table_mif[i].mem_clk,
				exynos5_busfreq_table_mif[i].volt);
		if (ret) {
			dev_err(dev, "Fail to add opp entries.\n");
			return ret;
		}
	}

	opp_disable(data->dev[PPMU_MIF], 107000);

	for (i = LV_0; i < LV_INT_END; i++) {
		ret = opp_add(data->dev[PPMU_INT], exynos5_busfreq_table_int[i].mem_clk,
				exynos5_busfreq_table_int[i].volt);
		if (ret) {
			dev_err(dev, "Fail to add opp entries.\n");
			return ret;
		}
	}

	data->target = exynos5250_target;
	data->get_table_index = exynos5250_get_table_index;
	data->monitor = exynos5250_monitor;
	data->busfreq_suspend = exynos5250_suspend;
	data->busfreq_resume = exynos5250_resume;
	data->sampling_rate = usecs_to_jiffies(100000);

	data->table[PPMU_MIF] = exynos5_busfreq_table_mif;
	data->table[PPMU_INT] = exynos5_busfreq_table_int;

	/* Find max frequency for mif */
	data->max_freq[PPMU_MIF] =
			opp_get_freq(opp_find_freq_floor(data->dev[PPMU_MIF], &maxfreq));
	data->min_freq[PPMU_MIF] =
			opp_get_freq(opp_find_freq_ceil(data->dev[PPMU_MIF], &minfreq));
	data->curr_freq[PPMU_MIF] =
			opp_get_freq(opp_find_freq_ceil(data->dev[PPMU_MIF], &cdrexfreq));
	/* Find max frequency for int */
	maxfreq = ULONG_MAX;
	minfreq = 0;
	data->max_freq[PPMU_INT] =
			opp_get_freq(opp_find_freq_floor(data->dev[PPMU_INT], &maxfreq));
	data->min_freq[PPMU_INT] =
			opp_get_freq(opp_find_freq_ceil(data->dev[PPMU_INT], &minfreq));
	data->curr_freq[PPMU_INT] =
			opp_get_freq(opp_find_freq_ceil(data->dev[PPMU_INT], &lrbusfreq));

	data->vdd_reg[PPMU_INT] = regulator_get(NULL, "vdd_int");
	if (IS_ERR(data->vdd_reg[PPMU_INT])) {
		pr_err("failed to get resource %s\n", "vdd_int");
		return -ENODEV;
	}

	data->vdd_reg[PPMU_MIF] = regulator_get(NULL, "vdd_mif");
	if (IS_ERR(data->vdd_reg[PPMU_MIF])) {
		pr_err("failed to get resource %s\n", "vdd_mif");
		regulator_put(data->vdd_reg[PPMU_INT]);
		return -ENODEV;
	}

	if (!pop) {
		regulator_set_voltage(data->vdd_reg[PPMU_MIF], 1000000, 1000000);
		regulator_set_voltage(data->vdd_reg[PPMU_INT], exynos5_busfreq_table_int[LV_0].volt,
				exynos5_busfreq_table_int[LV_0].volt);
		regulator_put(data->vdd_reg[PPMU_MIF]);
		regulator_put(data->vdd_reg[PPMU_INT]);
		data->vdd_reg[PPMU_MIF] = ERR_PTR(-ENODEV);
		data->vdd_reg[PPMU_INT] = ERR_PTR(-ENODEV);
		return -ENODEV;
	}

	/* Request min 300MHz */
	dev_lock(dev, dev, MIF_LOCK_LCD);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->busfreq_early_suspend_handler.suspend = &busfreq_early_suspend;
	data->busfreq_early_suspend_handler.resume = &busfreq_late_resume;
	data->busfreq_early_suspend_handler.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 2;
	register_early_suspend(&data->busfreq_early_suspend_handler);
#endif
	if (samsung_rev() < EXYNOS5250_REV_1_0) {
		tmp = __raw_readl(EXYNOS5_ABBG_INT_CONTROL);
		tmp &= ~(0x1f | (1 << 31) | (1 << 7));
		tmp |= ((8 + INT_RBB) | (1 << 31) | (1 << 7));
		__raw_writel(tmp, EXYNOS5_ABBG_INT_CONTROL);
	}

	init_done = true;
	return 0;
}
