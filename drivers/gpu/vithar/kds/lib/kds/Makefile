#
# (C) COPYRIGHT 2011-2012 ARM Limited. All rights reserved.
#
# This program is free software and is provided to you under the terms of the GNU General Public License version 2
# as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
#
# A copy of the licence is included with the program, and can also be obtained from Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA. 
#
#


# linux build system bootstrap for out-of-tree module

# default to building for the host
ARCH ?= $(shell uname -m)
CONFIG_KDS_TEST ?= n
ifneq ($(VITHAR_ROOT),)
EXTRA_CFLAGS="-I$(KDS_PATH)/../../include"
SRC+=kds.c
else

ifeq ($(KDIR),)
$(error Must specify KDIR to point to the kernel to target))
endif

all:
	$(MAKE) ARCH=$(ARCH) -C $(KDIR) M=$(CURDIR) EXTRA_CFLAGS="-I$(CURDIR)/../../include" CONFIG_KDS=m CONFIG_KDS_TEST=$(CONFIG_KDS_TEST)

clean:
	$(MAKE) ARCH=$(ARCH) -C $(KDIR) M=$(CURDIR) clean

endif
