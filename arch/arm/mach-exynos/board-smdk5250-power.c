/* linux/arch/arm/mach-exynos/board-smdk5250-power.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include <asm/io.h>

#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/pd.h>
#include <plat/gpio-cfg.h>

#include <mach/ppmu.h>
#include <mach/dev.h>
#include <mach/regs-pmu.h>
#include <mach/irqs.h>

#include <linux/mfd/s5m87xx/s5m-core.h>
#include <linux/mfd/s5m87xx/s5m-pmic.h>

#if defined(CONFIG_EXYNOS_THERMAL)
#include <mach/tmu.h>
#endif

#include "board-smdk5250.h"

#define REG_INFORM4            (S5P_INFORM4)

/* S5M8767 Regulator */
static int s5m_cfg_irq(void)
{
	/* AP_PMIC_IRQ: EINT26 */
	s3c_gpio_cfgpin(EXYNOS5_GPX3(2), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS5_GPX3(2), S3C_GPIO_PULL_UP);
	return 0;
}

static struct regulator_consumer_supply s5m8767_ldo4_consumer =
	REGULATOR_SUPPLY("vdd_ldo4", NULL);

static struct regulator_consumer_supply s5m8767_buck1_consumer =
	REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply s5m8767_buck2_consumer =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply s5m8767_buck3_consumer =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply s5m8767_buck4_consumer =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_init_data s5m8767_buck1_data = {
	.constraints	= {
		.name		= "vdd_mif range",
		.min_uV		=  950000,
		.max_uV		= 1300000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck1_consumer,
};

static struct regulator_init_data s5m8767_buck2_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		=  800000,
		.max_uV		= 1350000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck2_consumer,
};

static struct regulator_init_data s5m8767_buck3_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		=  900000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck3_consumer,
};

static struct regulator_init_data s5m8767_buck4_data = {
	.constraints	= {
		.name		= "vdd_g3d range",
		.min_uV		=  700000,
		.max_uV		= 1300000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck4_consumer,
};

static struct regulator_init_data s5m8767_ldo4_data = {
	.constraints	= {
		.name		= "vdd_ldo4",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_ldo4_consumer,
};

static struct s5m_regulator_data gaia_regulators[] = {
	{S5M8767_LDO4, &s5m8767_ldo4_data},
	{S5M8767_BUCK1, &s5m8767_buck1_data},
	{S5M8767_BUCK2, &s5m8767_buck2_data},
	{S5M8767_BUCK3, &s5m8767_buck3_data},
	{S5M8767_BUCK4, &s5m8767_buck4_data},
};

struct s5m_opmode_data s5m8767_opmode_data[S5M8767_REG_MAX] = {
	[S5M8767_LDO4] = {S5M8767_LDO4, S5M_OPMODE_STANDBY},
	[S5M8767_BUCK1] = {S5M8767_BUCK1, S5M_OPMODE_STANDBY},
	[S5M8767_BUCK2] = {S5M8767_BUCK2, S5M_OPMODE_STANDBY},
	[S5M8767_BUCK3] = {S5M8767_BUCK3, S5M_OPMODE_STANDBY},
	[S5M8767_BUCK4] = {S5M8767_BUCK4, S5M_OPMODE_STANDBY},
};

static struct s5m_platform_data smdk5250_s5m8767_pdata = {
	.device_type		= S5M8767X,
	.irq_base		= IRQ_BOARD_START,
	.num_regulators		= ARRAY_SIZE(gaia_regulators),
	.regulators		= gaia_regulators,
	.cfg_pmic_irq		= s5m_cfg_irq,
	.wakeup			= 1,
	.opmode_data		= s5m8767_opmode_data,
	.wtsr_smpl		= 1,

	.buck_default_idx	= 1,
	.buck_gpios[0]		= EXYNOS5_GPD1(0),
	.buck_gpios[1]		= EXYNOS5_GPD1(1),
	.buck_gpios[2]		= EXYNOS5_GPD1(2),

	.buck_ds[0]		= EXYNOS5_GPX2(3),
	.buck_ds[1]		= EXYNOS5_GPX2(4),
	.buck_ds[2]		= EXYNOS5_GPX2(5),

	.buck_ramp_delay        = 25,
	.buck2_ramp_enable      = true,
	.buck3_ramp_enable      = true,
	.buck4_ramp_enable      = true,

	.buck2_init		= 1200000,
	.buck3_init		= 1200000,
	.buck4_init		= 1200000,
};
/* End of S5M8767 */

static struct i2c_board_info i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("s5m87xx", 0xCC >> 1),
		.platform_data = &smdk5250_s5m8767_pdata,
		.irq		= IRQ_EINT(26),
	},
};

#ifdef CONFIG_BATTERY_SAMSUNG
static struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id	= -1,
};
#endif

#ifdef CONFIG_BUSFREQ_OPP
/* BUSFREQ to control memory/bus*/
static struct device_domain busfreq;
#endif

static struct platform_device exynos5_busfreq = {
	.id = -1,
	.name = "exynos-busfreq",
};

#ifdef CONFIG_EXYNOS_THERMAL
/* below temperature base on the celcius degree */
struct tmu_data exynos_tmu_data __initdata = {
	.ts = {
		.stop_throttle  = 82,
		.start_throttle = 85,
		.stop_warning  = 95,
		.start_warning = 103,
		.start_tripping = 110,		/* temp to do tripping */
		.start_hw_tripping = 113,       /* temp to do hw_trpping*/
#ifdef CONFIG_TC_VOLTAGE
		.stop_tc = 13,
		.start_tc = 10,
#endif
#ifdef CONFIG_MIF_VC
		.stop_mif_vc = 27,
		.start_mif_vc = 25,
#endif
	},
	.cpulimit = {
		.throttle_freq = 800000,
		.warning_freq = 200000,
	},
#ifdef CONFIG_TC_VOLTAGE
	.temp_compensate = {
		.arm_volt = 925000, /* vdd_arm in uV for temperature compensation */
		.bus_volt = 900000,
	},
#endif
	.efuse_value = 55,
	.slope = 0x10008802,
	.mode = 0,
};
#endif

static struct platform_device *smdk5250_power_devices[] __initdata = {
	/* Samsung Power Domain */
	&exynos5_device_pd[PD_MFC],
	&exynos5_device_pd[PD_G3D],
	&exynos5_device_pd[PD_ISP],
	&exynos5_device_pd[PD_GSCL],
	&exynos5_device_pd[PD_DISP1],
#ifdef CONFIG_SND_SAMSUNG_I2S
	&exynos5_device_pd[PD_MAUDIO],
#endif
	&s3c_device_i2c0,
#ifdef CONFIG_BATTERY_SAMSUNG
	&samsung_device_battery,
#endif
#ifdef CONFIG_EXYNOS_THERMAL
	&exynos_device_tmu,
#endif
	&exynos5_busfreq,
};

static int smdk5250_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	int mode = 0;

	if ((code == SYS_RESTART) && _cmd)
		if (!strcmp((char *)_cmd, "recovery"))
			mode = 0xf;

	__raw_writel(mode, REG_INFORM4);

	return NOTIFY_DONE;
}

static struct notifier_block smdk5250_reboot_notifier = {
	.notifier_call = smdk5250_notifier_call,
};

void __init exynos5_smdk5250_power_init(void)
{
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

#if defined(CONFIG_EXYNOS_DEV_PD) && defined(CONFIG_PM_RUNTIME)
	exynos_pd_enable(&exynos5_device_pd[PD_MFC].dev);
	exynos_pd_enable(&exynos5_device_pd[PD_G3D].dev);
	exynos_pd_enable(&exynos5_device_pd[PD_ISP].dev);
	exynos_pd_enable(&exynos5_device_pd[PD_GSCL].dev);
	exynos_pd_enable(&exynos5_device_pd[PD_DISP1].dev);
	exynos_pd_enable(&exynos5_device_pd[PD_MAUDIO].dev);
#elif defined(CONFIG_EXYNOS_DEV_PD)
	/*
	 * These power domains should be always on
	 * without runtime pm support.
	 */
	exynos_pd_enable(&exynos5_device_pd[PD_MFC].dev);
	exynos_pd_enable(&exynos5_device_pd[PD_G3D].dev);
	exynos_pd_enable(&exynos5_device_pd[PD_ISP].dev);
	exynos_pd_enable(&exynos5_device_pd[PD_GSCL].dev);
	exynos_pd_enable(&exynos5_device_pd[PD_DISP1].dev);
	exynos_pd_enable(&exynos5_device_pd[PD_MAUDIO].dev);
#endif

#ifdef CONFIG_EXYNOS_THERMAL
	exynos_tmu_set_platdata(&exynos_tmu_data);
#endif

#ifdef CONFIG_BUSFREQ_OPP
	dev_add(&busfreq, &exynos5_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_CPU], &exynos5_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DDR_C], &exynos5_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DDR_R1], &exynos5_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DDR_L], &exynos5_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_RIGHT0_BUS], &exynos5_busfreq.dev);
#endif
	register_reboot_notifier(&smdk5250_reboot_notifier);

	platform_add_devices(smdk5250_power_devices,
			ARRAY_SIZE(smdk5250_power_devices));
}
