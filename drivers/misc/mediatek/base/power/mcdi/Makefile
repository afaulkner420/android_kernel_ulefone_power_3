#
# Copyright (C) 2017 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.
#

ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/mcdi/include/

obj-$(CONFIG_MACH_MT6763) += mcdi_v1/
obj-$(CONFIG_MACH_MT6763) += mtk_mcdi_governor.o
obj-$(CONFIG_MACH_MT6763) += mtk_mcdi_governor_hint.o
obj-$(CONFIG_MACH_MT6758) += mcdi_v1/
obj-$(CONFIG_MACH_MT6758) += mtk_mcdi_governor.o
obj-$(CONFIG_MACH_MT6758) += mtk_mcdi_governor_hint.o
