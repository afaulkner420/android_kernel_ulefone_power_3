# Copyright (C) 2016 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.

ifdef MTK_PLATFORM
DRVGEN_PATH := drivers/misc/mediatek/dws/$(MTK_PLATFORM)
DTB_PATH := $(srctree)/arch/$(SRCARCH)/boot/dts

ifeq ($(strip $(CONFIG_ARM64)), y)
DTB_NAMES := $(addprefix $(DTB_PATH)/, $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES)))
else
DTB_NAMES := $(addprefix $(DTB_PATH)/, $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES)))
endif

ifndef DRVGEN_OUT
DRVGEN_OUT := $(objtree)/arch/$(SRCARCH)/boot/dts
endif
export DRVGEN_OUT

ALL_DRVGEN_FILE := $(MTK_PROJECT)/cust.dtsi

DWS_FILE := $(srctree)/$(DRVGEN_PATH)/$(MTK_PROJECT).dws

ifneq ($(wildcard $(DWS_FILE)),)
DRVGEN_FILE_LIST := $(addprefix $(DRVGEN_OUT)/,$(ALL_DRVGEN_FILE))
else
DRVGEN_FILE_LIST :=
endif

DRVGEN_TOOL := $(srctree)/tools/dct/DrvGen.py
DRVGEN_FIG := $(wildcard $(dir $(DRVGEN_TOOL))config/*.fig)

.PHONY: drvgen
drvgen: $(DRVGEN_FILE_LIST)
$(DRVGEN_FILE_LIST): $(DRVGEN_TOOL) $(DWS_FILE) $(DRVGEN_FIG)
	for i in $(DTB_NAMES); do \
		base_prj=`grep -m 1 "#include [<\"].*\/cust\.dtsi[>\"]" $$i.dts | sed 's/#include [<"]//g' | sed 's/\/cust\.dtsi[>"]//g'`;\
		prj_path=$(DRVGEN_OUT)/$$base_prj ;\
		mkdir -p $$prj_path ;\
		$(python) $(DRVGEN_TOOL) $(srctree)/$(DRVGEN_PATH)/$$base_prj.dws $$prj_path $$prj_path cust_dtsi;\
	done

endif#MTK_PLATFORM
