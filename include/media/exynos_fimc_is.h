/* linux/arch/arm/plat-s5p/include/plat/fimc_is.h
 *
 * Copyright (C) 2011 Samsung Electronics, Co. Ltd
 *
 * Exynos 4 series FIMC-IS slave device support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef EXYNOS_FIMC_IS_H_
#define EXYNOS_FIMC_IS_H_ __FILE__

#include <linux/videodev2.h>

#define FIMC_IS_MAX_CAMIF_CLIENTS	2
#define EXYNOS4_FIMC_IS_MAX_DIV_CLOCKS		2
#define EXYNOS4_FIMC_IS_MAX_CONTROL_CLOCKS	16

#define UART_ISP_SEL		0
#define UART_ISP_RATIO		1

#define to_fimc_is_plat(d)	(to_platform_device(d)->dev.platform_data)

#if defined(CONFIG_ARCH_EXYNOS5)
enum exynos5_csi_id {
	CSI_ID_A = 0,
	CSI_ID_B
};

enum exynos5_flite_id {
	FLITE_ID_A = 0,
	FLITE_ID_B
};

enum exynos5_sensor_position {
	SENSOR_POSITION_REAR = 0,
	SENSOR_POSITION_FRONT
};
enum exynos5_sensor_id {
	SENSOR_NAME_S5K3H2	= 1,
	SENSOR_NAME_S5K6A3	= 2,
	SENSOR_NAME_S5K4E5	= 3,
	SENSOR_NAME_S5K3H7	= 4,
	SENSOR_NAME_CUSTOM	= 5,
	SENSOR_NAME_END
};

enum exynos5_sensor_channel {
	SENSOR_CONTROL_I2C0	= 0,
	SENSOR_CONTROL_I2C1	= 1
};
#endif

struct platform_device;

#if defined(CONFIG_ARCH_EXYNOS5)
/**
 * struct exynos4_fimc_is_sensor_info  - image sensor information required for host
 *			      interace configuration.
 *
 * @board_info: pointer to I2C subdevice's board info
 * @clk_frequency: frequency of the clock the host interface provides to sensor
 * @bus_type: determines bus type, MIPI, ITU-R BT.601 etc.
 * @csi_data_align: MIPI-CSI interface data alignment in bits
 * @i2c_bus_num: i2c control bus id the sensor is attached to
 * @mux_id: FIMC camera interface multiplexer index (separate for MIPI and ITU)
 * @flags: flags defining bus signals polarity inversion (High by default)
*/
struct exynos4_fimc_is_sensor_info {

};
#endif
/**
 * struct exynos4_platform_fimc_is - camera host interface platform data
 *
 * @isp_info: properties of camera sensor required for host interface setup
*/
struct exynos4_platform_fimc_is {
	int	hw_ver;
	struct clk	*div_clock[EXYNOS4_FIMC_IS_MAX_DIV_CLOCKS];
	struct clk	*control_clock[EXYNOS4_FIMC_IS_MAX_CONTROL_CLOCKS];
	void	(*cfg_gpio)(struct platform_device *pdev);
	int	(*clk_get)(struct platform_device *pdev);
	int	(*clk_put)(struct platform_device *pdev);
	int	(*clk_cfg)(struct platform_device *pdev);
	int	(*clk_on)(struct platform_device *pdev);
	int	(*clk_off)(struct platform_device *pdev);
};

#if defined(CONFIG_ARCH_EXYNOS5)
struct exynos5_platform_fimc_is {
	int	hw_ver;
	struct exynos4_fimc_is_sensor_info
		*sensor_info[FIMC_IS_MAX_CAMIF_CLIENTS];
	void	(*cfg_gpio)(struct platform_device *pdev);
	int	(*clk_cfg)(struct platform_device *pdev);
	int	(*clk_on)(struct platform_device *pdev);
	int	(*clk_off)(struct platform_device *pdev);
};
#endif

extern struct exynos4_platform_fimc_is exynos4_fimc_is_default_data;
extern void exynos4_fimc_is_set_platdata(struct exynos4_platform_fimc_is *pd);
#if defined(CONFIG_ARCH_EXYNOS5)
extern void exynos5_fimc_is_set_platdata(struct exynos5_platform_fimc_is *pd);
#endif
/* defined by architecture to configure gpio */
extern void exynos_fimc_is_cfg_gpio(struct platform_device *pdev);

/* platform specific clock functions */
extern int exynos_fimc_is_cfg_clk(struct platform_device *pdev);
extern int exynos_fimc_is_clk_on(struct platform_device *pdev);
extern int exynos_fimc_is_clk_off(struct platform_device *pdev);
extern int exynos_fimc_is_clk_get(struct platform_device *pdev);
extern int exynos_fimc_is_clk_put(struct platform_device *pdev);

#if defined(CONFIG_ARCH_EXYNOS5)
/* defined by architecture to configure gpio */
extern void exynos5_fimc_is_cfg_gpio(struct platform_device *pdev);

/* platform specific clock functions */
extern int exynos5_fimc_is_cfg_clk(struct platform_device *pdev);
extern int exynos5_fimc_is_clk_on(struct platform_device *pdev);
extern int exynos5_fimc_is_clk_off(struct platform_device *pdev);
#endif
#endif /* EXYNOS_FIMC_IS_H_ */
