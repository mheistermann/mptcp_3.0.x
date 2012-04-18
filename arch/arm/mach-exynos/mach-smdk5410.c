/* linux/arch/arm/mach-exynos/mach-smdk5410.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/serial_core.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/exynos5.h>
#include <plat/cpu.h>
#include <plat/clock.h>

#include <mach/map.h>

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDK5410_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDK5410_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDK5410_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdk5410_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDK5410_UCON_DEFAULT,
		.ulcon		= SMDK5410_ULCON_DEFAULT,
		.ufcon		= SMDK5410_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDK5410_UCON_DEFAULT,
		.ulcon		= SMDK5410_ULCON_DEFAULT,
		.ufcon		= SMDK5410_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDK5410_UCON_DEFAULT,
		.ulcon		= SMDK5410_ULCON_DEFAULT,
		.ufcon		= SMDK5410_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDK5410_UCON_DEFAULT,
		.ulcon		= SMDK5410_ULCON_DEFAULT,
		.ufcon		= SMDK5410_UFCON_DEFAULT,
	},
};

static struct platform_device *smdk5410_devices[] __initdata = {
};

static void __init smdk5410_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdk5410_uartcfgs, ARRAY_SIZE(smdk5410_uartcfgs));
}

static void __init smdk5410_machine_init(void)
{
	platform_add_devices(smdk5410_devices, ARRAY_SIZE(smdk5410_devices));
}

MACHINE_START(SMDK5410, "SMDK5410")
	.boot_params	= EXYNOS5_PA_SDRAM + 0x100,
	.init_irq	= exynos5_init_irq,
	.map_io		= smdk5410_map_io,
	.init_machine	= smdk5410_machine_init,
	.timer		= &exynos4_timer,
MACHINE_END
