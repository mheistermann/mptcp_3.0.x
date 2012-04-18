/*
 *
 * (C) COPYRIGHT 2010 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

/**
 * @file mali_uk_os.h
 * User-Kernel Interface (kernel and user-side) dependent APIs (Linux).
 */

#ifndef _UK_OS_H_ /* Linux */
#define _UK_OS_H_

#ifdef MALI_DEBUG
#define MALI_UK_CANARY_VALUE    0xb2bdbdf6
#endif

#if MALI_BACKEND_KERNEL

#define LINUX_UK_BASE_MAGIC 0x80 /* BASE UK ioctl */

struct uku_context
{
	struct
	{
#ifdef MALI_DEBUG
		u32 canary;
#endif
		int fd;
	} ukup_internal_struct;
};

#else /* MALI_BACKEND_KERNEL */

typedef struct ukk_userspace
{
	void * ctx;
	mali_error (*dispatch)(void * /*ctx*/, void* /*msg*/, u32 /*size*/);
	void (*close)(struct ukk_userspace * /*self*/);
} ukk_userspace;

typedef ukk_userspace * (*kctx_open)(void);

struct uku_context
{
	struct
	{
#ifdef MALI_DEBUG
		u32 canary;
#endif
		ukk_userspace * ukku;
	} ukup_internal_struct;
};

#endif /* MALI_BACKEND_KERNEL */

#endif /* _UK_OS_H_ */
