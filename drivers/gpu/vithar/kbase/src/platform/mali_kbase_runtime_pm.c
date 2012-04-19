/* drivers/gpu/vithar/kbase/src/platform/mali_kbase_runtime_pm.c
 *
 * Copyright 2011 by S.LSI. Samsung Electronics Inc.
 * San#24, Nongseo-Dong, Giheung-Gu, Yongin, Korea
 *
 * Samsung SoC Mali-T604 runtime pm driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

/**
 * @file mali_kbase_runtime_pm.c
 * Runtime PM
 */

#include <osk/mali_osk.h>
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_uku.h>
#include <kbase/src/common/mali_kbase_mem.h>
#include <kbase/src/common/mali_midg_regmap.h>
#include <kbase/src/linux/mali_kbase_mem_linux.h>
#include <uk/mali_ukk.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/semaphore.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <kbase/src/platform/mali_kbase_platform.h>
#include <kbase/src/platform/mali_kbase_runtime_pm.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>

#define RUNTIME_PM_DELAY_TIME 100
static int allow_rp_control= 0;

/** Suspend callback from the OS.
 *
 * This is called by Linux runtime PM when the device should suspend.
 *
 * @param dev	The device to suspend
 *
 * @return A standard Linux error code
 */
int kbase_device_runtime_idle(struct device *dev)
{
	return 1;
}
int kbase_device_runtime_suspend(struct device *dev)
{
	if(allow_rp_control==0) {
		return 0;
	}
#if MALI_RTPM_DEBUG
	printk("kbase_device_runtime_suspend\n");
#endif
	return kbase_platform_cmu_pmu_control(dev, 0);
}

/** Resume callback from the OS.
 *
 * This is called by Linux runtime PM when the device should resume from suspension.
 *
 * @param dev	The device to resume
 *
 * @return A standard Linux error code
 */
int kbase_device_runtime_resume(struct device *dev)
{
	if(allow_rp_control==0) {
		return 0;
	}
#if MALI_RTPM_DEBUG
	printk("kbase_device_runtime_resume\n");
#endif
	return kbase_platform_cmu_pmu_control(dev, 1);
}

/** Disable runtime pm
 *
 * @param dev	The device to enable rpm
 *
 * @return A standard Linux error code
 */
void kbase_device_runtime_disable(struct device *dev)
{
	pm_runtime_disable(dev);
}

/** Initialize runtiem pm fields in given device 
 *
 * @param dev	The device to initialize
 *
 * @return A standard Linux error code
 */
extern void pm_runtime_init(struct device *dev);


void kbase_device_runtime_allow_rp_control(void)
{
	allow_rp_control = 1;
}

void kbase_device_runtime_init(struct device *dev)
{
	pm_runtime_init(dev);
	pm_suspend_ignore_children(dev, true);
	pm_runtime_enable(dev);
}

void kbase_device_runtime_get_sync(struct device *dev)
{
	int result;
	struct kbase_device *kbdev;
	kbdev = dev_get_drvdata(dev);

	/********************************************
	 *
	 *  This is workaround about occurred kernel panic when you turn off the system.
	 *
	 *  System kernel will call the "__pm_runtime_disable" when you turn off the system.
	 *  After that function, System kernel do not run the runtimePM API more.
	 *
	 *  So, this code is check the "dev->power.disable_depth" value is not zero.
	 *
	********************************************/
	if(dev->power.disable_depth > 0) {
		if(kbdev->pm.cmu_pmu_status == 0)
			kbase_platform_cmu_pmu_control(dev, 1);
		return;
	}

	result = pm_runtime_resume(dev);

#if MALI_RTPM_DEBUG
	//printk( "kbase_device_runtime_get_sync, usage_count=%d, runtime_status=%d\n", atomic_read(&dev->power.usage_count), dev->power.runtime_status);
	printk( "kbase_device_runtime_get_sync, usage_count=%d\n", atomic_read(&dev->power.usage_count));
#endif

	/********************************************
	 *
	 *  This check is re-confirm about maybe context switch by another cpu when turn off the system.
	 *
	 *  runtimePM put_sync -------- runtimePM get_sync -------- runtimePM put_sync          : CPU 0
	 *                                      \
	 *                                       \ ( context running by another core. )
	 *                                        \
	 *                                         - (turn off the system) runtimePM disable    : CPU 1
	 *                                                                    \
	 *                                                                     \
	 *                                                                      => do not success implement runtimePM API
	********************************************/
	if(result < 0 && result == -EAGAIN)
		kbase_platform_cmu_pmu_control(dev, 1);
	else if(result < 0)
		OSK_PRINT_ERROR(OSK_BASE_PM, "pm_runtime_get_sync failed (%d)\n", result);
}

void kbase_device_runtime_put_sync(struct device *dev)
{
	int result;
	/********************************************
	 *
	 *  This is workaround about occurred kernel panic when you turn off the system.
	 *
	 *  System kernel will call the "__pm_runtime_disable" when you turn off the system.
	 *  After that function, System kernel do not run the runtimePM API more.
	 *
	 *  So, this code is check the "dev->power.disable_depth" value is not zero.
	 *
	********************************************/
	if(dev->power.disable_depth > 0)
		return;

	result = pm_schedule_suspend(dev, RUNTIME_PM_DELAY_TIME);

#if MALI_RTPM_DEBUG
	//printk( "kbase_device_runtime_put_sync, usage_count=%d, runtime_status=%d\n", atomic_read(&dev->power.usage_count), dev->power.runtime_status);
	printk( "kbase_device_runtime_put_sync, usage_count=%d\n", atomic_read(&dev->power.usage_count));
#endif

	if(result < 0 && result != -EAGAIN)
		OSK_PRINT_ERROR(OSK_BASE_PM, "pm_runtime_put_sync failed (%d)\n", result);
}
