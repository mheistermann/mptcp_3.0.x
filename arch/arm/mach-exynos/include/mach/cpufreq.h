/* linux/arch/arm/mach-exynos/include/mach/cpufreq.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - CPUFreq support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/* CPU frequency level index for using cpufreq lock API
 * This should be same with cpufreq_frequency_table
*/

enum cpufreq_level_index {
	L0, L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11, L12, L13,
};

enum cpufreq_level_request {
	CPU_L0,		/* 1400MHz */
	CPU_L1,		/* 1200MHz */
	CPU_L2,		/* 1000MHz */
	CPU_L3,		/* 800MHz */
	CPU_L4,		/* 500MHz */
	CPU_L5,		/* 200MHz */
	CPU_LEVEL_END,
};

enum busfreq_level_request {
	BUS_L0,		/* MEM 400MHz BUS 200MHz */
	BUS_L1,		/* MEM 267MHz BUS 160MHz */
	BUS_L2,		/* MEM 133MHz BUS 133MHz */
	BUS_LEVEL_END,
};

enum cpufreq_lock_ID {
	DVFS_LOCK_ID_G2D,	/* G2D */
	DVFS_LOCK_ID_TV,	/* TV */
	DVFS_LOCK_ID_MFC,	/* MFC */
	DVFS_LOCK_ID_USB,	/* USB */
	DVFS_LOCK_ID_CAM,	/* CAM */
	DVFS_LOCK_ID_PM,	/* PM */
	DVFS_LOCK_ID_USER,	/* USER */
	DVFS_LOCK_ID_TMU,	/* TMU */
	DVFS_LOCK_ID_LPA,	/* LPA */
	DVFS_LOCK_ID_END,
};

int exynos4_cpufreq_lock(unsigned int nId,
			enum cpufreq_level_request cpufreq_level);
void exynos4_cpufreq_lock_free(unsigned int nId);

int exynos4_busfreq_lock(unsigned int nId,
			enum busfreq_level_request busfreq_level);
void exynos4_busfreq_lock_free(unsigned int nId);

int exynos4_cpufreq_upper_limit(unsigned int nId,
			enum cpufreq_level_request cpufreq_level);
void exynos4_cpufreq_upper_limit_free(unsigned int nId);

#define MAX_INDEX	10

struct exynos_dvfs_info {
	unsigned long	mpll_freq_khz;
	unsigned int	pll_safe_idx;
	unsigned int	pm_lock_idx;
	unsigned int	max_support_idx;
	unsigned int	min_support_idx;
	struct clk	*cpu_clk;
	unsigned int	*volt_table;
	struct cpufreq_frequency_table	*freq_table;
	void (*set_freq)(unsigned int, unsigned int);
	bool (*need_apll_change)(unsigned int, unsigned int);
};

#define SUPPORT_1400MHZ	(1<<31)
#define SUPPORT_1200MHZ	(1<<30)
#define SUPPORT_1000MHZ	(1<<29)
#define SUPPORT_FREQ_SHIFT	29
#define SUPPORT_FREQ_MASK	7

extern int exynos4210_cpufreq_init(struct exynos_dvfs_info *);
extern int exynos4212_cpufreq_init(struct exynos_dvfs_info *);