#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

MTK_PLATFORM_PATH_ROOT := $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)
MTK_PROJECT_PATH_ROOT := $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)

CONFIG_MTK_PLATFORM_SENSOR := $(CONFIG_MTK_PLATFORM)

# Vanzo:yucheng on: Sat, 15 Oct 2016 12:31:11 +0800
# added for aosp management to import our variable
project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))
ifneq ($(strip $(project_name)),)
-include $(srctree)/../zprojects/$(project_name)/$(project_name).mk
ifneq ($(strip $(CUSTOM_KERNEL_IMGSENSOR)),)
CONFIG_CUSTOM_KERNEL_IMGSENSOR:=$(CUSTOM_KERNEL_IMGSENSOR)
endif
endif
# End of Vanzo: yucheng
subdir-ccflags-y += -I$(MTK_PROJECT_PATH_ROOT)/camera/camera \

MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/camera/$(MTK_PLATFORM) \
             -I$(MTK_CUSTOM_PATH)/imgsensor/inc \
             -I$(MTK_CUSTOM_PATH)/imgsensor/src/$(CONFIG_MTK_PLATFORM_SENSOR) \
             -I$(MTK_CUSTOM_PATH)/cam_cal/inc \
             -I$(MTK_CUSTOM_PATH)/smi \
             -I$(MTK_CUSTOM_PATH)/include/mt-plat/$(MTK_PLATFORM)/include/mach \


CUSTOM_KERNEL_FLAGS := $(addprefix -D, $(shell echo \
             $(CONFIG_CUSTOM_KERNEL_IMGSENSOR) | tr a-z A-Z))
subdir-ccflags-y += $(CUSTOM_KERNEL_FLAGS)

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM_SENSOR))/

