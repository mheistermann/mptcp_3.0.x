/* linux/arch/arm/mach-exynos/include/mach/asv.h
 *
 * copyright (c) 2012 samsung electronics co., ltd.
 *		http://www.samsung.com/
 *
 * exynos - asv header file
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the gnu general public license version 2 as
 * published by the free software foundation.
*/

#ifndef __ASM_ARCH_NEW_ASV_H
#define __ASM_ARCH_NEW_ASV_H __FILE__

#define ASV_GRP_NR(_id)		_id##_ASV_GRP_NR
#define DVFS_LEVEL_NR(_id)	_id##_DVFS_LEVEL_NR
#define MAX_VOLT(_id)		_id##_MAX_VOLT

enum asv_type_id {
	ID_ARM,
	ID_INT,
	ID_MIF,
	ID_G3D,
};

/* define Struct for ASV common */
struct asv_common {
	const char	*lot_name;
	unsigned int	ids_value;
	unsigned int	hpm_value;
	unsigned int	(*init)(void);
	unsigned int	(*regist_asv_member)(void);
};

struct asv_volt_table {
	unsigned int	asv_freq;
	unsigned int	asv_volt;
};

/* define struct for information of each ASV type */
struct asv_info {
	struct list_head	node;
	enum asv_type_id	asv_type;
	const char		*name;
	struct asv_ops		*ops;
	unsigned int		asv_group_nr;
	unsigned int		dvfs_level_nr;
	unsigned int		result_asv_grp;
	unsigned int		max_volt_value;
	struct asv_volt_table	*asv_volt;
};

/* Operation for ASV*/
struct asv_ops {
	unsigned int	(*get_asv_group)(struct asv_common *asv_comm);
	void		(*set_asv_info)(struct asv_info *asv_inform, bool show_volt);
};

/* define function for common asv */
extern void add_asv_member(struct asv_info *exynos_asv_info);
extern struct asv_info *asv_get(enum asv_type_id exynos_asv_type_id);
extern unsigned int get_match_volt(enum asv_type_id target_type, unsigned int target_freq);

/* define function for initialize of SoC */

#endif /* __ASM_ARCH_NEW_ASV_H */
