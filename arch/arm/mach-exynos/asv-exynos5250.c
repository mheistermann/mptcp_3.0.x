/* linux/arch/arm/mach-exynos/asv-exynos5250.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS5250 - ASV(Adoptive Support Voltage) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <mach/asv-exynos.h>
#include <mach/asv-exynos5250.h>
#include <mach/map.h>
#include <mach/regs-pmu5.h>

#include <plat/cpu.h>

#include "board-smdk5250.h"

#define CHIP_ID_REG		(S5P_VA_CHIPID + 0x4)
#define MIF_ASV_REG		(S5P_VA_CHIPID + 0x8)
#define LOT_ID_REG		(S5P_VA_CHIPID + 0x14)

/* ASV function for Fused Chip */
#define IDS_ARM_OFFSET		24
#define IDS_ARM_MASK		0xFF
#define HPM_OFFSET		12
#define HPM_MASK		0x1F

#define LOT_ID_OFFSET		11
#define LOT_ID_MASK		0x1FFFFF
#define LOT_ID_LEN		5

/* ASV function for Speed group Fused Chip */
#define SG_FUSE_OFFSET		3
#define SG_FUSE_MASK		0x1

#define G1_ORG_SG_OFFSET	17
#define G1_ORG_SG_MASK		0xF
#define G1_MOD_SG_OFFSET	21
#define G1_MOD_SG_MASK		0x7

#define G2_ORG_SG_OFFSET	26
#define G2_ORG_SG_MASK		0x3
#define G2_MOD_SG_OFFSET	28
#define G2_MOD_SG_MASK		0x3

bool exynos5250_is_use_sg_fused(void)
{
	unsigned int tmp;

	tmp = __raw_readl(CHIP_ID_REG);

	return (tmp >> SG_FUSE_OFFSET) & SG_FUSE_MASK;
}

void exynos5250_set_abb(struct asv_info *asv_inform)
{
	void __iomem *target_reg;
	unsigned int target_value = asv_inform->abb_info->target_abb;

	switch (asv_inform->asv_type) {
	case ID_ARM:
		target_reg = EXYNOS5_ABB_MEMBER(ABB_ARM);
		break;
	case ID_INT:
		target_reg = EXYNOS5_ABB_MEMBER(ABB_INT);
		break;
	case ID_MIF:
		target_reg = EXYNOS5_ABB_MEMBER(ABB_MIF);
		break;
	case ID_G3D:
		target_reg = EXYNOS5_ABB_MEMBER(ABB_G3D);
		break;
	default:
		return;
	}

	set_abb(target_reg, target_value);
}

struct abb_common exynos5250_abb_info = {
	.set_target_abb	= exynos5250_set_abb,
};

char *special_lot_list[] = {
	"NZVPU",
	"NZVR7",
};

bool is_special_lot;

unsigned int exynos5250_check_lot_id(const char *target_lot_id)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(special_lot_list); i++) {
		if (!strncmp(target_lot_id, special_lot_list[i], LOT_ID_LEN)) {
			pr_info("Exynos5250 : Lot ID is %s\n", target_lot_id);
			return 1;
		}
	}

	return 0;
}

static unsigned int exynos5250_get_asv_group_arm(struct asv_common *asv_comm)
{
	unsigned int i;
	unsigned int refer_ids;
	unsigned int refer_hpm;
	unsigned int asv_group = 0;
	unsigned int group_shift = 0;
	struct asv_info *target_asv_info = asv_get(ID_ARM);

	if (exynos5250_is_use_sg_fused()) {
		asv_group = (__raw_readl(CHIP_ID_REG) >> G1_ORG_SG_OFFSET) & G1_ORG_SG_MASK;
		group_shift = (__raw_readl(CHIP_ID_REG) >> G1_MOD_SG_OFFSET) & G1_MOD_SG_MASK;

		asv_group -= group_shift;

		if (asv_group < 0)
			asv_group = 0;
	} else {
		for (i = 0; i < target_asv_info->asv_group_nr; i++) {
			if (is_special_lot) {
				refer_ids = arm_org_refer_table_get_asv[0][i];
				refer_hpm = arm_org_refer_table_get_asv[1][i];
			} else {
				refer_ids = arm_refer_table_get_asv[0][i];
				refer_hpm = arm_refer_table_get_asv[1][i];
			}

			if ((asv_comm->ids_value <= refer_ids) || (asv_comm->hpm_value <= refer_hpm)) {
				asv_group = i;
				break;
			}
		}
	}

	return asv_group;
}

bool is_pmic_max;

static void exynos5250_set_asv_info_arm(struct asv_info *asv_inform, bool show_volt)
{
	unsigned int i;
	unsigned int target_freq;
	unsigned int target_volt;
	unsigned int target_asv_grp_nr = asv_inform->result_asv_grp;

	asv_inform->asv_volt = kmalloc((sizeof(struct asv_volt_table) * asv_inform->dvfs_level_nr), GFP_KERNEL);

	for (i = 0; i < asv_inform->dvfs_level_nr; i++) {
		if (is_special_lot) {
			target_freq = arm_org_asv_volt_info[i][0];
			target_volt = arm_org_asv_volt_info[i][target_asv_grp_nr + 1];
		} else {
			if (is_pmic_max)	{
				target_freq = arm_asv_volt_info_max[i][0];
				target_volt = arm_asv_volt_info_max[i][target_asv_grp_nr + 1];
			} else {
				target_freq = arm_asv_volt_info_s5m8767[i][0];
				target_volt = arm_asv_volt_info_s5m8767[i][target_asv_grp_nr + 1];
			}
		}

		asv_inform->asv_volt[i].asv_freq = target_freq;
		asv_inform->asv_volt[i].asv_volt = target_volt;
	}

	if (show_volt) {
		for (i = 0; i < asv_inform->dvfs_level_nr; i++)
			pr_info("%s LV%d freq : %d volt : %d\n",
					asv_inform->name, i,
					asv_inform->asv_volt[i].asv_freq,
					asv_inform->asv_volt[i].asv_volt);
	}

	/* Set ABB value */
	if (is_special_lot)
		asv_inform->abb_info->target_abb = arm_org_abb_ref_table[target_asv_grp_nr];
	else
		asv_inform->abb_info->target_abb = arm_abb_ref_table[target_asv_grp_nr];
}

struct asv_ops exynos5250_asv_ops_arm = {
	.get_asv_group	= exynos5250_get_asv_group_arm,
	.set_asv_info	= exynos5250_set_asv_info_arm,
};

static unsigned int exynos5250_get_asv_group_int(struct asv_common *asv_comm)
{
	unsigned int i;
	unsigned int refer_ids;
	unsigned int refer_hpm;
	unsigned int asv_group = 0;
	unsigned int group_shift = 0;
	struct asv_info *target_asv_info = asv_get(ID_INT);

	if (exynos5250_is_use_sg_fused()) {
		asv_group = (__raw_readl(CHIP_ID_REG) >> G1_ORG_SG_OFFSET) & G1_ORG_SG_MASK;
		group_shift = (__raw_readl(CHIP_ID_REG) >> G1_MOD_SG_OFFSET) & G1_MOD_SG_MASK;

		asv_group -= group_shift;

		if (asv_group < 0)
			asv_group = 0;
	} else {
		for (i = 0; i < target_asv_info->asv_group_nr; i++) {
			if (is_special_lot) {
				refer_ids = int_org_refer_table_get_asv[0][i];
				refer_hpm = int_org_refer_table_get_asv[1][i];
			} else {
				refer_ids = int_refer_table_get_asv[0][i];
				refer_hpm = int_refer_table_get_asv[1][i];
			}

			if ((asv_comm->ids_value <= refer_ids) || (asv_comm->hpm_value <= refer_hpm)) {
				asv_group = i;
				break;
			}
		}
	}

	return asv_group;
}

static void exynos5250_set_asv_info_int(struct asv_info *asv_inform, bool show_volt)
{
	unsigned int i;
	unsigned int target_freq;
	unsigned int target_volt;
	unsigned int target_asv_grp_nr = asv_inform->result_asv_grp;

	asv_inform->asv_volt = kmalloc((sizeof(struct asv_volt_table) * asv_inform->dvfs_level_nr), GFP_KERNEL);

	for (i = 0; i < asv_inform->dvfs_level_nr; i++) {
		if (is_special_lot) {
			target_freq = int_org_asv_volt_info[i][0];
			target_volt = int_org_asv_volt_info[i][target_asv_grp_nr + 1];
		} else {
			if (is_pmic_max)	{
				target_freq = int_asv_volt_info_max[i][0];
				target_volt = int_asv_volt_info_max[i][target_asv_grp_nr + 1];
			} else {
				target_freq = int_asv_volt_info_s5m8767[i][0];
				target_volt = int_asv_volt_info_s5m8767[i][target_asv_grp_nr + 1];
			}
		}

		asv_inform->asv_volt[i].asv_freq = target_freq;
		asv_inform->asv_volt[i].asv_volt = target_volt;
	}

	if (show_volt) {
		for (i = 0; i < asv_inform->dvfs_level_nr; i++)
			pr_info("%s LV%d freq : %d volt : %d\n",
					asv_inform->name, i,
					asv_inform->asv_volt[i].asv_freq,
					asv_inform->asv_volt[i].asv_volt);
	}

	/* Set ABB value */
	if (is_special_lot)
		asv_inform->abb_info->target_abb = int_org_abb_ref_table[target_asv_grp_nr];
	else
		asv_inform->abb_info->target_abb = int_abb_ref_table[target_asv_grp_nr];
}

struct asv_ops exynos5250_asv_ops_int = {
	.get_asv_group	= exynos5250_get_asv_group_int,
	.set_asv_info	= exynos5250_set_asv_info_int,
};

static unsigned int exynos5250_get_asv_group_mif(struct asv_common *asv_comm)
{
	unsigned int i;
	unsigned int refer_ids;
	unsigned int refer_hpm;
	unsigned int asv_group = 0;
	unsigned int group_shift = 0;
	struct asv_info *target_asv_info = asv_get(ID_MIF);

	if (exynos5250_is_use_sg_fused()) {
		asv_group = (__raw_readl(MIF_ASV_REG) >> G2_ORG_SG_OFFSET) & G2_ORG_SG_MASK;
		group_shift = (__raw_readl(MIF_ASV_REG) >> G2_MOD_SG_OFFSET) & G2_MOD_SG_MASK;

		asv_group -= group_shift;

		if (asv_group < 0)
			asv_group = 0;
	} else {
		for (i = 0; i < target_asv_info->asv_group_nr; i++) {
			if (is_special_lot) {
				refer_ids = mif_org_refer_table_get_asv[0][i];
				refer_hpm = mif_org_refer_table_get_asv[1][i];

				if ((asv_comm->ids_value <= refer_ids) || (asv_comm->hpm_value <= refer_hpm)) {
					asv_group = i;
					break;
				}
			} else {
				refer_hpm = mif_refer_table_get_asv[0][i];

				if (asv_comm->hpm_value <= refer_hpm) {
					asv_group = i;
					break;
				}
			}
		}
	}

	return asv_group;
}

static void exynos5250_set_asv_info_mif(struct asv_info *asv_inform, bool show_volt)
{
	unsigned int i;
	unsigned int target_freq;
	unsigned int target_volt;
	unsigned int target_asv_grp_nr = asv_inform->result_asv_grp;

	asv_inform->asv_volt = kmalloc((sizeof(struct asv_volt_table) * asv_inform->dvfs_level_nr), GFP_KERNEL);

	for (i = 0; i < asv_inform->dvfs_level_nr; i++) {
		if (is_special_lot) {
			target_freq = mif_org_asv_volt_info[i][0];
			target_volt = mif_org_asv_volt_info[i][target_asv_grp_nr + 1];
		} else {
			if (is_pmic_max)	{
				target_freq = mif_asv_volt_info_max[i][0];
				target_volt = mif_asv_volt_info_max[i][target_asv_grp_nr + 1];
			} else {
				target_freq = mif_asv_volt_info_s5m8767[i][0];
				target_volt = mif_asv_volt_info_s5m8767[i][target_asv_grp_nr + 1];
			}
		}

		asv_inform->asv_volt[i].asv_freq = target_freq;
		asv_inform->asv_volt[i].asv_volt = target_volt;
	}

	if (show_volt) {
		for (i = 0; i < asv_inform->dvfs_level_nr; i++)
			pr_info("%s LV%d freq : %d volt : %d\n",
					asv_inform->name, i,
					asv_inform->asv_volt[i].asv_freq,
					asv_inform->asv_volt[i].asv_volt);
	}

	/* Set ABB value */
	if (is_special_lot)
		asv_inform->abb_info->target_abb = mif_org_abb_ref_table[target_asv_grp_nr];
	else
		asv_inform->abb_info->target_abb = mif_abb_ref_table[target_asv_grp_nr];
}

struct asv_ops exynos5250_asv_ops_mif = {
	.get_asv_group	= exynos5250_get_asv_group_mif,
	.set_asv_info	= exynos5250_set_asv_info_mif,
};

static unsigned int exynos5250_get_asv_group_g3d(struct asv_common *asv_comm)
{
	unsigned int i;
	unsigned int refer_ids;
	unsigned int refer_hpm;
	unsigned int group_shift = 0;
	unsigned int asv_group = 0;
	struct asv_info *target_asv_info = asv_get(ID_G3D);

	if (exynos5250_is_use_sg_fused()) {
		asv_group = (__raw_readl(CHIP_ID_REG) >> G1_ORG_SG_OFFSET) & G1_ORG_SG_MASK;
		group_shift = (__raw_readl(CHIP_ID_REG) >> G1_MOD_SG_OFFSET) & G1_MOD_SG_MASK;

		asv_group -= group_shift;

		if (asv_group < 0)
			asv_group = 0;
	} else {
		for (i = 0; i < target_asv_info->asv_group_nr; i++) {
			if (is_special_lot) {
				refer_ids = g3d_org_refer_table_get_asv[0][i];
				refer_hpm = g3d_org_refer_table_get_asv[1][i];
			} else {
				refer_ids = g3d_refer_table_get_asv[0][i];
				refer_hpm = g3d_refer_table_get_asv[1][i];
			}

			if ((asv_comm->ids_value <= refer_ids) || (asv_comm->hpm_value <= refer_hpm)) {
				asv_group = i;
				break;
			}
		}
	}

	return asv_group;
}

static void exynos5250_set_asv_info_g3d(struct asv_info *asv_inform, bool show_volt)
{
	unsigned int i;
	unsigned int target_freq;
	unsigned int target_volt;
	unsigned int target_asv_grp_nr = asv_inform->result_asv_grp;

	asv_inform->asv_volt = kmalloc((sizeof(struct asv_volt_table) * asv_inform->dvfs_level_nr), GFP_KERNEL);

	for (i = 0; i < asv_inform->dvfs_level_nr; i++) {
		if (is_special_lot) {
			target_freq = g3d_org_asv_volt_info[i][0];
			target_volt = g3d_org_asv_volt_info[i][target_asv_grp_nr + 1];
		} else {
			if (is_pmic_max)	{
				target_freq = g3d_asv_volt_info_max[i][0];
				target_volt = g3d_asv_volt_info_max[i][target_asv_grp_nr + 1];
			} else {
				target_freq = g3d_asv_volt_info_s5m8767[i][0];
				target_volt = g3d_asv_volt_info_s5m8767[i][target_asv_grp_nr + 1];
			}
		}

		asv_inform->asv_volt[i].asv_freq = target_freq;
		asv_inform->asv_volt[i].asv_volt = target_volt;
	}

	if (show_volt) {
		for (i = 0; i < asv_inform->dvfs_level_nr; i++)
			pr_info("%s LV%d freq : %d volt : %d\n",
					asv_inform->name, i,
					asv_inform->asv_volt[i].asv_freq,
					asv_inform->asv_volt[i].asv_volt);
	}

	/* Set ABB value */
	if (is_special_lot)
		asv_inform->abb_info->target_abb = g3d_org_abb_ref_table[target_asv_grp_nr];
	else
		asv_inform->abb_info->target_abb = g3d_abb_ref_table[target_asv_grp_nr];
}

struct asv_ops exynos5250_asv_ops_g3d = {
	.get_asv_group	= exynos5250_get_asv_group_g3d,
	.set_asv_info	= exynos5250_set_asv_info_g3d,
};

struct asv_info exynos5250_asv_member[] = {
	{
		.asv_type	= ID_ARM,
		.name		= "VDD_ARM",
		.ops		= &exynos5250_asv_ops_arm,
		.asv_group_nr	= ASV_GRP_NR(ARM),
		.dvfs_level_nr	= DVFS_LEVEL_NR(ARM),
		.max_volt_value	= MAX_VOLT(ARM),
		.abb_info	= &exynos5250_abb_info,
	}, {
		.asv_type	= ID_INT,
		.name		= "VDD_INT",
		.ops		= &exynos5250_asv_ops_int,
		.asv_group_nr	= ASV_GRP_NR(INT),
		.dvfs_level_nr	= DVFS_LEVEL_NR(INT),
		.max_volt_value	= MAX_VOLT(INT),
		.abb_info	= &exynos5250_abb_info,
	}, {
		.asv_type	= ID_MIF,
		.name		= "VDD_MIF",
		.ops		= &exynos5250_asv_ops_mif,
		.asv_group_nr	= ASV_GRP_NR(MIF),
		.dvfs_level_nr	= DVFS_LEVEL_NR(MIF),
		.max_volt_value	= MAX_VOLT(MIF),
		.abb_info	= &exynos5250_abb_info,
	}, {
		.asv_type	= ID_G3D,
		.name		= "VDD_G3D",
		.ops		= &exynos5250_asv_ops_g3d,
		.asv_group_nr	= ASV_GRP_NR(G3D),
		.dvfs_level_nr	= DVFS_LEVEL_NR(G3D),
		.max_volt_value	= MAX_VOLT(G3D),
		.abb_info	= &exynos5250_abb_info,
	},
};

struct asv_info exynos5250_org_asv_member[] = {
	{
		.asv_type	= ID_ARM,
		.name		= "VDD_ARM",
		.ops		= &exynos5250_asv_ops_arm,
		.asv_group_nr	= ASV_GRP_NR(ARM_ORG),
		.dvfs_level_nr	= DVFS_LEVEL_NR(ARM_ORG),
		.max_volt_value	= MAX_VOLT(ARM_ORG),
		.abb_info	= &exynos5250_abb_info,
	}, {
		.asv_type	= ID_INT,
		.name		= "VDD_INT",
		.ops		= &exynos5250_asv_ops_int,
		.asv_group_nr	= ASV_GRP_NR(INT_ORG),
		.dvfs_level_nr	= DVFS_LEVEL_NR(INT_ORG),
		.max_volt_value	= MAX_VOLT(INT_ORG),
		.abb_info	= &exynos5250_abb_info,
	}, {
		.asv_type	= ID_MIF,
		.name		= "VDD_MIF",
		.ops		= &exynos5250_asv_ops_mif,
		.asv_group_nr	= ASV_GRP_NR(MIF_ORG),
		.dvfs_level_nr	= DVFS_LEVEL_NR(MIF_ORG),
		.max_volt_value	= MAX_VOLT(MIF_ORG),
		.abb_info	= &exynos5250_abb_info,
	}, {
		.asv_type	= ID_G3D,
		.name		= "VDD_G3D",
		.ops		= &exynos5250_asv_ops_g3d,
		.asv_group_nr	= ASV_GRP_NR(G3D_ORG),
		.dvfs_level_nr	= DVFS_LEVEL_NR(G3D_ORG),
		.max_volt_value	= MAX_VOLT(G3D_ORG),
		.abb_info	= &exynos5250_abb_info,
	},
};

static unsigned int exynos5250_regist_asv_member(void)
{
	unsigned int i;

	/* Regist asv member into list */
	if (is_special_lot)
		for (i = 0; i < ARRAY_SIZE(exynos5250_org_asv_member); i++)
			add_asv_member(&exynos5250_org_asv_member[i]);
	else
		for (i = 0; i < ARRAY_SIZE(exynos5250_asv_member); i++) {
			if (exynos5250_is_use_sg_fused())
				exynos5250_asv_member[i].use_sg_fused = true;

			add_asv_member(&exynos5250_asv_member[i]);
		}

	return 0;
}

int exynos5250_init_asv(struct asv_common *asv_info)
{
	unsigned int tmp;
	unsigned int i;
	unsigned int lid_reg;
	unsigned int rev_lid = 0;
	char lot_id[LOT_ID_LEN];

	is_special_lot = false;

	/* read IDS and HPM value from  CHIP ID */
	tmp = __raw_readl(CHIP_ID_REG);

	asv_info->hpm_value = (tmp >> HPM_OFFSET) & HPM_MASK;
	asv_info->ids_value = (tmp >> IDS_ARM_OFFSET) & IDS_ARM_MASK;

	/* read LOT ID */
	lid_reg = __raw_readl(LOT_ID_REG);

	for (i = 0; i < 32; i++) {
		tmp = (lid_reg >> i) & 0x1;
		rev_lid += tmp << (31 - i);
	}

	lot_id[0] = 'N';
	lid_reg = (rev_lid >> LOT_ID_OFFSET) & LOT_ID_MASK;

	for (i = 4; i >= 1; i--) {
		tmp = lid_reg % 36;
		lid_reg /= 36;
		lot_id[i] = (tmp < 10) ? (tmp + '0') : ((tmp - 10) + 'A');
	}

	asv_info->lot_name = lot_id;

	is_special_lot = exynos5250_check_lot_id(lot_id);

	tmp = get_smdk5250_regulator();
	switch (tmp)	{
	case SMDK5250_REGULATOR_MAX77686:
	case SMDK5250_REGULATOR_MAX8997:
		is_pmic_max = true;
		pr_info("EXYNOS ASV PMIC MAXIM Detected!\n");
		break;
	case SMDK5250_REGULATOR_S5M8767:
		is_pmic_max = false;
		pr_info("EXYNOS ASV PMIC S5M Detected!\n");
		break;
	}

	if (!exynos5250_is_use_sg_fused())
		pr_info("EXYNOS5250 ASV : %s IDS : %d HPM : %d\n", asv_info->lot_name,
					asv_info->ids_value, asv_info->hpm_value);

	asv_info->regist_asv_member = exynos5250_regist_asv_member;

	return 0;
}
