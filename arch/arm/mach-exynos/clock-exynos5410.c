/* linux/arch/arm/mach-exynos/clock-exynos5410.c
 *
 * Copyright (c) 2010-2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS5410 - Clock support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/syscore_ops.h>

#include <plat/cpu-freq.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/pll.h>
#include <plat/s5p-clock.h>
#include <plat/clock-clksrc.h>
#include <plat/devs.h>
#include <plat/pm.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-audss.h>
#include <mach/sysmmu.h>
#include <mach/exynos-clock.h>
#include <mach/clock-domain.h>

static int exynos5_clk_ip_peric_ctrl(struct clk *clk, int enable)
{
	return s5p_gatectrl(EXYNOS5_CLKGATE_IP_PERIC, clk, enable);
}

static int exynos5_clksrc_mask_peric0_ctrl(struct clk *clk, int enable)
{
	return s5p_gatectrl(EXYNOS5_CLKSRC_MASK_PERIC0, clk, enable);
}

static int exynos5_clk_ip_gen_ctrl(struct clk *clk, int enable)
{
	return s5p_gatectrl(EXYNOS5_CLKGATE_IP_GEN, clk, enable);
}

static int exynos5_clk_ip_fsys_ctrl(struct clk *clk, int enable)
{
	return s5p_gatectrl(EXYNOS5_CLKGATE_IP_FSYS, clk, enable);
}

static int exynos5_clk_bus_fsys0_ctrl(struct clk *clk, int enable)
{
	return s5p_gatectrl(EXYNOS5_CLKGATE_BUS_FSYS0, clk, enable);
}

static int exynos5_clk_ip_g2d_ctrl(struct clk *clk, int enable)
{
	return s5p_gatectrl(EXYNOS5_CLKGATE_IP_G2D, clk, enable);
}
/*
 * Clock for PHY
 */
static struct clk exynos5_clk_sclk_hdmi24m = {
	.name		= "sclk_hdmi24m",
	.rate		= 24000000,
};

static struct clk exynos5_clk_sclk_hdmi27m = {
	.name		= "sclk_hdmi27m",
	.rate		= 27000000,
};

static struct clk exynos5_clk_sclk_hdmiphy = {
	.name		= "sclk_hdmiphy",
};

static struct clk exynos5_clk_sclk_dptxphy = {
	.name		= "sclk_dptx",
};

static struct clk exynos5_clk_sclk_usbphy = {
	.name		= "sclk_usbphy",
	.rate		= 48000000,
};

/*
 * PLL output clock
 */
static struct clk clk_fout_bpll = {
	.name		= "fout_bpll",
	.id		= -1,
};

static struct clk clk_fout_cpll = {
	.name		= "fout_cpll",
	.id		= -1,
};

static struct clk clk_fout_kpll = {
	.name		= "fout_kpll",
	.id		= -1,
};

static struct clk clk_fout_ipll = {
	.name		= "fout_ipll",
	.id		= -1,
};

/*
 * Mux of PLL output clock
 */
/* Mux output for APLL */
static struct clksrc_clk exynos5_clk_mout_apll = {
	.clk	= {
		.name		= "mout_apll",
	},
	.sources = &clk_src_apll,
	.reg_src = { .reg = EXYNOS5_CLKSRC_CPU, .shift = 0, .size = 1 },
};

/* Mux output for MPLL */
struct clksrc_clk exynos5_clk_mout_mpll = {
	.clk = {
		.name		= "mout_mpll",
	},
	.sources = &clk_src_mpll,
	.reg_src = { .reg = EXYNOS5_CLKSRC_CORE1, .shift = 8, .size = 1 },
};

/* Mux output for EPLL */
static struct clksrc_clk exynos5_clk_mout_epll = {
	.clk	= {
		.name		= "mout_epll",
	},
	.sources = &clk_src_epll,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP2, .shift = 12, .size = 1 },
};

/* Mux output for BPLL */
static struct clk *clk_src_bpll_list[] = {
	[0] = &clk_fin_bpll,
	[1] = &clk_fout_bpll,
};

static struct clksrc_sources clk_src_bpll = {
	.sources	= clk_src_bpll_list,
	.nr_sources	= ARRAY_SIZE(clk_src_bpll_list),
};

static struct clksrc_clk exynos5_clk_mout_bpll = {
	.clk	= {
		.name		= "mout_bpll",
	},
	.sources = &clk_src_bpll,
	.reg_src = { .reg = EXYNOS5_CLKSRC_CDREX, .shift = 0, .size = 1 },
};

/* Mux output for CPLL */
static struct clk *clk_src_cpll_list[] = {
	[0] = &clk_fin_cpll,
	[1] = &clk_fout_cpll,
};

static struct clksrc_sources clk_src_cpll = {
	.sources	= clk_src_cpll_list,
	.nr_sources	= ARRAY_SIZE(clk_src_cpll_list),
};

static struct clksrc_clk exynos5_clk_mout_cpll = {
	.clk	= {
		.name		= "mout_cpll",
	},
	.sources = &clk_src_cpll,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP2, .shift = 8, .size = 1 },
};

/* Mux output for KPLL */
static struct clk *clk_src_kpll_list[] = {
	[0] = &clk_fin_kpll,
	[1] = &clk_fout_kpll,
};

static struct clksrc_sources clk_src_kpll = {
	.sources	= clk_src_kpll_list,
	.nr_sources	= ARRAY_SIZE(clk_src_kpll_list),
};

static struct clksrc_clk exynos5_clk_mout_kpll = {
	.clk	= {
		.name		= "mout_kpll",
	},
	.sources = &clk_src_kpll,
	.reg_src = { .reg = EXYNOS5_SRC_KFC, .shift = 0, .size = 1 },
};

/* Mux output for IPLL */
static struct clk *clk_src_ipll_list[] = {
	[0] = &clk_fin_ipll,
	[1] = &clk_fout_ipll,
};

static struct clksrc_sources clk_src_ipll = {
	.sources	= clk_src_ipll_list,
	.nr_sources	= ARRAY_SIZE(clk_src_ipll_list),
};

static struct clksrc_clk exynos5_clk_mout_ipll = {
	.clk	= {
		.name		= "mout_ipll",
	},
	.sources = &clk_src_ipll,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP2, .shift = 14, .size = 1 },
};

/* Mux output for VPLL_SRC */
static struct clk *exynos5_clkset_mout_vpllsrc_list[] = {
	[0] = &clk_fin_vpll,
	[1] = &exynos5_clk_sclk_hdmi27m,
};

static struct clksrc_sources exynos5_clkset_mout_vpllsrc = {
	.sources	= exynos5_clkset_mout_vpllsrc_list,
	.nr_sources	= ARRAY_SIZE(exynos5_clkset_mout_vpllsrc_list),
};

static struct clksrc_clk exynos5_clk_mout_vpllsrc = {
	.clk	= {
		.name		= "vpll_src",
	},
	.sources = &exynos5_clkset_mout_vpllsrc,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP2, .shift = 0, .size = 1 },
};

/* Mux output for VPLL */
static struct clk *exynos5_clkset_mout_vpll_list[] = {
	[0] = &exynos5_clk_mout_vpllsrc.clk,
	[1] = &clk_fout_vpll,
};

static struct clksrc_sources exynos5_clkset_mout_vpll = {
	.sources	= exynos5_clkset_mout_vpll_list,
	.nr_sources	= ARRAY_SIZE(exynos5_clkset_mout_vpll_list),
};

static struct clksrc_clk exynos5_clk_mout_vpll = {
	.clk	= {
		.name		= "mout_vpll",
	},
	.sources = &exynos5_clkset_mout_vpll,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP2, .shift = 16, .size = 1 },
};

/* CPU(EAGLE & KFC) Part clock */
static struct clk *exynos5_clkset_mout_cpu_list[] = {
	[0] = &exynos5_clk_mout_apll.clk,
	[1] = &exynos5_clk_mout_mpll.clk,
};

static struct clksrc_sources exynos5_clkset_mout_cpu = {
	.sources	= exynos5_clkset_mout_cpu_list,
	.nr_sources	= ARRAY_SIZE(exynos5_clkset_mout_cpu_list),
};

static struct clksrc_clk exynos5_clk_mout_cpu = {
	.clk	= {
		.name		= "mout_cpu",
	},
	.sources = &exynos5_clkset_mout_cpu,
	.reg_src = { .reg = EXYNOS5_CLKSRC_CPU, .shift = 16, .size = 1 },
};

static struct clksrc_clk exynos5_clk_dout_arm = {
	.clk	= {
		.name		= "dout_arm",
		.parent		= &exynos5_clk_mout_cpu.clk,
	},
	.reg_div = { .reg = EXYNOS5_CLKDIV_CPU0, .shift = 0, .size = 3 },
};

static struct clksrc_clk exynos5_clk_dout_arm2 = {
	.clk	= {
		.name		= "dout_arm2",
		.parent		= &exynos5_clk_dout_arm.clk,
	},
	.reg_div = { .reg = EXYNOS5_CLKDIV_CPU0, .shift = 28, .size = 3 },
};

static struct clksrc_clk exynos5_clk_dout_acp = {
	.clk	= {
		.name		= "dout_acp",
		.parent		= &exynos5_clk_dout_arm2.clk,
	},
	.reg_div = { .reg = EXYNOS5_CLKDIV_CPU0, .shift = 8, .size = 3 },
};

static struct clk exynos5_clk_armclk = {
	.name		= "armclk",
	.parent		= &exynos5_clk_dout_acp.clk,
};

static struct clksrc_clk exynos5_clk_dout_kfc = {
	.clk	= {
		.name		= "dout_kfc",
		.parent		= &exynos5_clk_mout_kpll.clk,
	},
	.reg_div = { .reg = EXYNOS5_DIV_KFC0, .shift = 0, .size = 3 },
};

static struct clk exynos5_clk_kfcclk = {
	.name		= "kfcclk",
	.parent		= &exynos5_clk_dout_kfc.clk,
};

/* CDREX Part clock */
static struct clksrc_clk exynos5_clk_dout_sclk_cdrex = {
	.clk	= {
		.name		= "dout_sclk_cdrex",
		.parent		= &exynos5_clk_mout_bpll.clk,
	},
	.reg_div = { .reg = EXYNOS5_CLKDIV_CDREX, .shift = 24, .size = 3 },
};

static struct clksrc_clk exynos5_clk_dout_clk2x_phy = {
	.clk	= {
		.name		= "dout_clk2x_phy",
		.parent		= &exynos5_clk_dout_sclk_cdrex.clk,
	},
	.reg_div = { .reg = EXYNOS5_CLKDIV_CDREX, .shift = 3, .size = 5 },
};

static struct clksrc_clk exynos5_clk_dout_cclk_cdrex = {
	.clk	= {
		.name		= "dout_cclk_cdrex",
		.parent		= &exynos5_clk_dout_clk2x_phy.clk,
	},
	.reg_div = { .reg = EXYNOS5_CLKDIV_CDREX, .shift = 8, .size = 3 },
};

static struct clksrc_clk exynos5_clk_dout_pclk_cdrex = {
	.clk	= {
		.name		= "dout_pclk_cdrex",
		.parent		= &exynos5_clk_dout_cclk_cdrex.clk,
	},
	.reg_div = { .reg = EXYNOS5_CLKDIV_CDREX, .shift = 28, .size = 3 },
};

/* Top Part clock */

/* Mux output for MPLL_USER */
static struct clk *exynos5_clkset_mout_mpll_user_list[] = {
	[0] = &clk_fin_mpll,
	[1] = &exynos5_clk_mout_mpll.clk,
};

static struct clksrc_sources exynos5_clkset_mout_mpll_user = {
	.sources	= exynos5_clkset_mout_mpll_user_list,
	.nr_sources	= ARRAY_SIZE(exynos5_clkset_mout_mpll_user_list),
};

static struct clksrc_clk exynos5_clk_mout_mpll_user = {
	.clk	= {
		.name		= "mout_mpll_user",
	},
	.sources = &exynos5_clkset_mout_mpll_user,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP2, .shift = 20, .size = 1 },
};

/* Mux output for BPLL_USER */
static struct clk *exynos5_clkset_mout_bpll_user_list[] = {
	[0] = &clk_fin_bpll,
	[1] = &exynos5_clk_mout_bpll.clk,
};

static struct clksrc_sources exynos5_clkset_mout_bpll_user = {
	.sources	= exynos5_clkset_mout_bpll_user_list,
	.nr_sources	= ARRAY_SIZE(exynos5_clkset_mout_bpll_user_list),
};

static struct clksrc_clk exynos5_clk_mout_bpll_user = {
	.clk	= {
		.name		= "mout_bpll_user",
	},
	.sources = &exynos5_clkset_mout_bpll_user,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP2, .shift = 24, .size = 1 },
};

/* Mux output for MPLL_BPLL */
static struct clk *exynos5_clkset_mout_mpll_bpll_list[] = {
	[0] = &exynos5_clk_mout_mpll_user.clk,
	[1] = &exynos5_clk_mout_bpll_user.clk,
};

static struct clksrc_sources exynos5_clkset_mout_mpll_bpll = {
	.sources	= exynos5_clkset_mout_mpll_bpll_list,
	.nr_sources	= ARRAY_SIZE(exynos5_clkset_mout_mpll_bpll_list),
};

static struct clksrc_clk exynos5_clk_mout_mpll_bpll = {
	.clk	= {
		.name		= "mout_mpll_bpll",
	},
	.sources = &exynos5_clkset_mout_mpll_bpll,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP1, .shift = 20, .size = 1 },
};

/* Clock for ACLK_XXX */
static struct clk *exynos5_clkset_aclk_xxx_list[] = {
	[0] = &exynos5_clk_mout_mpll_user.clk,
	[1] = &exynos5_clk_mout_bpll_user.clk,
};

static struct clksrc_sources exynos5_clkset_aclk_xxx = {
	.sources	= exynos5_clkset_aclk_xxx_list,
	.nr_sources	= ARRAY_SIZE(exynos5_clkset_aclk_xxx_list),
};

static struct clk *exynos5_clkset_aclk_xxx_pre_list[] = {
	[0] = &exynos5_clk_mout_cpll.clk,
	[1] = &exynos5_clk_mout_mpll_user.clk,
};

static struct clksrc_sources exynos5_clkset_aclk_xxx_pre = {
	.sources	= exynos5_clkset_aclk_xxx_pre_list,
	.nr_sources	= ARRAY_SIZE(exynos5_clkset_aclk_xxx_pre_list),
};

/* ACKL_400 */
static struct clksrc_clk exynos5_clk_aclk_400 = {
	.clk	= {
		.name		= "aclk_400",
	},
	.sources = &exynos5_clkset_aclk_xxx,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP0, .shift = 20, .size = 1 },
	.reg_div = { .reg = EXYNOS5_CLKDIV_TOP0, .shift = 24, .size = 3 },
};

/* ACLK_333_PRE */
static struct clksrc_clk exynos5_clk_aclk_333_pre = {
	.clk	= {
		.name		= "aclk_333_pre",
	},
	.sources = &exynos5_clkset_aclk_xxx_pre,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP0, .shift = 16, .size = 1 },
	.reg_div = { .reg = EXYNOS5_CLKDIV_TOP0, .shift = 20, .size = 3 },
};

/* ACLK_333 */
static struct clk *exynos5_clkset_aclk_333_list[] = {
	[0] = &clk_ext_xtal_mux,
	[1] = &exynos5_clk_aclk_333_pre.clk,
};

static struct clksrc_sources exynos5_clkset_aclk_333 = {
	.sources	= exynos5_clkset_aclk_333_list,
	.nr_sources	= ARRAY_SIZE(exynos5_clkset_aclk_333_list),
};

static struct clksrc_clk exynos5_clk_aclk_333 = {
	.clk	= {
		.name		= "aclk_333",
	},
	.sources = &exynos5_clkset_aclk_333,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP3, .shift = 24, .size = 1 },
};

/* ACKL_266 */
static struct clksrc_clk exynos5_clk_aclk_266 = {
	.clk	= {
		.name		= "aclk_266",
		.parent		= &exynos5_clk_mout_mpll_user.clk,
	},
	.reg_div = { .reg = EXYNOS5_CLKDIV_TOP0, .shift = 16, .size = 3 },
};

/* ACLK_166 */
static struct clksrc_clk exynos5_clk_aclk_166 = {
	.clk	= {
		.name		= "aclk_166",
	},
	.sources = &exynos5_clkset_aclk_xxx_pre,
	.reg_src = { .reg = EXYNOS5_CLKSRC_TOP0, .shift = 8, .size = 1 },
	.reg_div = { .reg = EXYNOS5_CLKDIV_TOP0, .shift = 8, .size = 3 },
};

/* Common Clock Src group */
struct clk *exynos5_clkset_group_list[] = {
	[0] = &clk_ext_xtal_mux,
	[1] = NULL,
	[2] = &exynos5_clk_sclk_hdmi24m,
	[3] = &exynos5_clk_sclk_dptxphy,
	[4] = &exynos5_clk_sclk_usbphy,
	[5] = &exynos5_clk_sclk_hdmiphy,
	[6] = &exynos5_clk_mout_mpll_bpll.clk,
	[7] = &exynos5_clk_mout_epll.clk,
	[8] = &exynos5_clk_mout_vpll.clk,
	[9] = &exynos5_clk_mout_cpll.clk,
};

static struct clksrc_sources exynos5_clkset_group = {

	.sources	= exynos5_clkset_group_list,
	.nr_sources	= ARRAY_SIZE(exynos5_clkset_group_list),
};

static struct clk exynos5_init_clocks[] = {
	{
		.name		= "uart",
		.devname	= "s5pv210-uart.0",
		.enable		= exynos5_clk_ip_peric_ctrl,
		.ctrlbit	= (1 << 0),
	}, {
		.name		= "uart",
		.devname	= "s5pv210-uart.1",
		.enable		= exynos5_clk_ip_peric_ctrl,
		.ctrlbit	= (1 << 1),
	}, {
		.name		= "uart",
		.devname	= "s5pv210-uart.2",
		.enable		= exynos5_clk_ip_peric_ctrl,
		.ctrlbit	= (1 << 2),
	}, {
		.name		= "uart",
		.devname	= "s5pv210-uart.3",
		.enable		= exynos5_clk_ip_peric_ctrl,
		.ctrlbit	= (1 << 3),
	}, {
		.name		= "uart",
		.devname	= "s5pv210-uart.4",
		.enable		= exynos5_clk_ip_peric_ctrl,
		.ctrlbit	= (1 << 4),
	},
};

struct clk exynos5_init_dmaclocks[] = {
	{
		.name		= "pdma",
		.devname	= "s3c-pl330.0",
		.enable		= exynos5_clk_ip_gen_ctrl,
		.ctrlbit	= ((1 << 4) | (1 << 14)),
	}, {
		.name		= "pdma",
		.devname	= "s3c-pl330.1",
		.enable		= exynos5_clk_bus_fsys0_ctrl,
		.ctrlbit	= (1 << 1),
	}, {
		.name		= "pdma",
		.devname	= "s3c-pl330.2",
		.enable		= exynos5_clk_bus_fsys0_ctrl,
		.ctrlbit	= (1 << 2),
	}, {
		.name		= "pdma",
		.enable		= exynos5_clk_ip_g2d_ctrl,
		.ctrlbit	= ((1 << 1) | (1 << 8)),
	},
};

static struct clksrc_clk exynos5_clksrcs[] = {
	{
		.clk	= {
			.name		= "uclk1",
			.devname	= "s5pv210-uart.0",
			.enable		= exynos5_clksrc_mask_peric0_ctrl,
			.ctrlbit	= (1 << 0),
		},
		.sources = &exynos5_clkset_group,
		.reg_src = { .reg = EXYNOS5_CLKSRC_PERIC0, .shift = 0, .size = 4 },
		.reg_div = { .reg = EXYNOS5_CLKDIV_PERIC0, .shift = 0, .size = 4 },
	}, {
		.clk	= {
			.name		= "uclk1",
			.devname	= "s5pv210-uart.1",
			.enable		= exynos5_clksrc_mask_peric0_ctrl,
			.ctrlbit	= (1 << 4),
		},
		.sources = &exynos5_clkset_group,
		.reg_src = { .reg = EXYNOS5_CLKSRC_PERIC0, .shift = 4, .size = 4 },
		.reg_div = { .reg = EXYNOS5_CLKDIV_PERIC0, .shift = 4, .size = 4 },
	}, {
		.clk	= {
			.name		= "uclk1",
			.devname	= "s5pv210-uart.2",
			.enable		= exynos5_clksrc_mask_peric0_ctrl,
			.ctrlbit	= (1 << 8),
		},
		.sources = &exynos5_clkset_group,
		.reg_src = { .reg = EXYNOS5_CLKSRC_PERIC0, .shift = 8, .size = 4 },
		.reg_div = { .reg = EXYNOS5_CLKDIV_PERIC0, .shift = 8, .size = 4 },
	}, {
		.clk	= {
			.name		= "uclk1",
			.devname	= "s5pv210-uart.3",
			.enable		= exynos5_clksrc_mask_peric0_ctrl,
			.ctrlbit	= (1 << 12),
		},
		.sources = &exynos5_clkset_group,
		.reg_src = { .reg = EXYNOS5_CLKSRC_PERIC0, .shift = 12, .size = 4 },
		.reg_div = { .reg = EXYNOS5_CLKDIV_PERIC0, .shift = 12, .size = 4 },
	},
};

/* Clock initialization code */
static struct clksrc_clk *exynos5_sysclks[] = {
	&exynos5_clk_mout_apll,
	&exynos5_clk_mout_mpll,
	&exynos5_clk_mout_epll,
	&exynos5_clk_mout_bpll,
	&exynos5_clk_mout_cpll,
	&exynos5_clk_mout_kpll,
	&exynos5_clk_mout_ipll,
	&exynos5_clk_mout_vpllsrc,
	&exynos5_clk_mout_vpll,
	&exynos5_clk_mout_mpll_user,
	&exynos5_clk_mout_bpll_user,
	&exynos5_clk_mout_mpll_bpll,
	&exynos5_clk_aclk_400,
	&exynos5_clk_aclk_333_pre,
	&exynos5_clk_aclk_333,
	&exynos5_clk_aclk_266,
	&exynos5_clk_aclk_166,
	&exynos5_clk_mout_cpu,
	&exynos5_clk_dout_arm,
	&exynos5_clk_dout_arm2,
	&exynos5_clk_dout_acp,
	&exynos5_clk_dout_kfc,
	&exynos5_clk_dout_sclk_cdrex,
	&exynos5_clk_dout_clk2x_phy,
	&exynos5_clk_dout_cclk_cdrex,
	&exynos5_clk_dout_pclk_cdrex,
};

static struct clk *exynos5_clks[] __initdata = {
	&exynos5_clk_sclk_hdmi27m,
	&exynos5_clk_armclk,
	&clk_fout_bpll,
	&clk_fout_cpll,
	&clk_fout_kpll,
	&clk_fout_ipll,
	&exynos5_clk_kfcclk,
};

void __init_or_cpufreq exynos5_setup_clocks(void)
{
	struct clk *xtal_clk;
	unsigned long xtal;
	unsigned long apll;
	unsigned long kpll;
	unsigned long mpll;
	unsigned long bpll;
	unsigned long cpll;
	unsigned long vpll;
	unsigned long vpllsrc;
	unsigned long ipll;
	unsigned long dpll;
	unsigned long epll;
	unsigned long eagle_clk;
	unsigned long kfc_clk;
	unsigned long mclk_cdrex;

	printk(KERN_DEBUG "%s: registering clocks\n", __func__);

	xtal_clk = clk_get(NULL, "xtal");
	BUG_ON(IS_ERR(xtal_clk));

	xtal = clk_get_rate(xtal_clk);

	clk_put(xtal_clk);

	printk(KERN_DEBUG "%s: xtal is %ld\n", __func__, xtal);

	/* Set and check PLLs */
	apll = s5p_get_pll35xx(xtal, __raw_readl(EXYNOS5_APLL_CON0));
	kpll = s5p_get_pll35xx(xtal, __raw_readl(EXYNOS5_KPLL_CON0));
	mpll = s5p_get_pll35xx(xtal, __raw_readl(EXYNOS5_MPLL_CON0));
	bpll = s5p_get_pll35xx(xtal, __raw_readl(EXYNOS5_BPLL_CON0));
	cpll = s5p_get_pll35xx(xtal, __raw_readl(EXYNOS5_CPLL_CON0));
	vpllsrc = clk_get_rate(&exynos5_clk_mout_vpllsrc.clk);
	vpll = s5p_get_pll36xx(vpllsrc, __raw_readl(EXYNOS5_VPLL_CON0),
			__raw_readl(EXYNOS5_VPLL_CON1));
	ipll = s5p_get_pll35xx(xtal, __raw_readl(EXYNOS5410_IPLL_CON0));
	dpll = s5p_get_pll35xx(xtal, __raw_readl(EXYNOS5_DPLL_CON0));
	epll = s5p_get_pll36xx(xtal, __raw_readl(EXYNOS5_EPLL_CON0),
			__raw_readl(EXYNOS5_EPLL_CON1));

	clk_fout_bpll.rate = bpll;
	clk_fout_cpll.rate = cpll;
	clk_fout_mpll.rate = mpll;
	clk_fout_epll.rate = epll;
	clk_fout_vpll.rate = vpll;
	clk_fout_kpll.rate = kpll;
	clk_fout_ipll.rate = ipll;
	clk_fout_dpll.rate = dpll;

	eagle_clk = clk_get_rate(&exynos5_clk_armclk);
	kfc_clk = clk_get_rate(&exynos5_clk_kfcclk);
	mclk_cdrex = clk_get_rate(&exynos5_clk_dout_clk2x_phy.clk);

	printk(KERN_INFO "EXYNOS5: EAGLECLK=%ld, KFCCLK=%ld, CDREX=%ld\n",
			eagle_clk, kfc_clk, mclk_cdrex);
}

void __init exynos5_register_clocks(void)
{
	int ptr;

	s3c24xx_register_clocks(exynos5_clks, ARRAY_SIZE(exynos5_clks));

	for (ptr = 0; ptr < ARRAY_SIZE(exynos5_sysclks); ptr++)
		s3c_register_clksrc(exynos5_sysclks[ptr], 1);

	s3c_register_clksrc(exynos5_clksrcs, ARRAY_SIZE(exynos5_clksrcs));
	s3c_register_clocks(exynos5_init_clocks, ARRAY_SIZE(exynos5_init_clocks));

	s3c_register_clocks(exynos5_init_dmaclocks, ARRAY_SIZE(exynos5_init_dmaclocks));
	s3c_disable_clocks(exynos5_init_dmaclocks, ARRAY_SIZE(exynos5_init_dmaclocks));

	s3c_pwmclk_init();
}
