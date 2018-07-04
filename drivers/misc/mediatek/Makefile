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

#$(call all-subdir-src-or-makefile)
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
# Vanzo:yucheng on: Wed, 12 Nov 2014 21:08:36 +0800
# added for aosp management to import our variable
project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))
ifneq ($(strip $(project_name)),)
-include $(srctree)/../zprojects/$(project_name)/$(project_name).mk
VANZO_PROJECT_HEADERS:=$(srctree)/../zprojects/$(project_name)/kernel-headers
subdir-ccflags-y += -I$(VANZO_PROJECT_HEADERS)
endif
# End of Vanzo:yucheng
#subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/hibcore
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -I$(srctree)/drivers/mmc/host/mediatek/$(MTK_PLATFORM)
# Vanzo:yucheng on: Thu, 25 Dec 2014 15:02:51 +0800
# for compatible the old config usage,so here I use the custom config to replace the kernel config 
ifneq ($(strip $(project_name)),)
ifneq ($(strip $(CUSTOM_KERNEL_FLASHLIGHT)),)
CONFIG_CUSTOM_KERNEL_FLASHLIGHT:=$(CUSTOM_KERNEL_FLASHLIGHT)
endif
ifneq ($(strip $(CUSTOM_KERNEL_LCM)),)
CONFIG_CUSTOM_KERNEL_LCM:=$(CUSTOM_KERNEL_LCM)
endif
ifneq ($(strip $(CUSTOM_KERNEL_IMGSENSOR)),)
CONFIG_CUSTOM_KERNEL_IMGSENSOR:=$(CUSTOM_KERNEL_IMGSENSOR)
endif

ifneq ($(strip $(CUSTOM_KERNEL_HALL)),)
    obj-y += hall/
endif
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/dtv/),)
    ifneq ($(strip $(CUSTOM_KERNEL_DTV)),)
        obj-y	+= dtv/
    endif
endif
endif
$(info CONFIG_CUSTOM_KERNEL_GYROSCOPE $(CONFIG_CUSTOM_KERNEL_GYROSCOPE))
$(info CUSTOM_KERNEL_GYROSCOPE $(CUSTOM_KERNEL_GYROSCOPE))
$(info CONFIG_CUSTOM_KERNEL_MAGNETOMETER $(CONFIG_CUSTOM_KERNEL_MAGNETOMETER))
$(info CUSTOM_KERNEL_MAGNETOMETER $(CUSTOM_KERNEL_MAGNETOMETER))
$(info CONFIG_CUSTOM_KERNEL_ACCELEROMETER $(CONFIG_CUSTOM_KERNEL_ACCELEROMETER))
$(info CUSTOM_KERNEL_ACCELEROMETER $(CUSTOM_KERNEL_ACCELEROMETER))
# End of Vanzo:yucheng

obj-$(CONFIG_MTK_EMI_MPU)	+= emi_mpu/
obj-$(CONFIG_MTK_DCS) += mtkdcs/

obj-$(CONFIG_MTK_BASE_POWER) += base/
obj-$(CONFIG_MEDIATEK_SOLUTION) += sched/
#obj-$(CONFIG_MEDIATEK_SOLUTION) += mach/
obj-$(CONFIG_MTK_SERIAL)	+= uart/
obj-$(CONFIG_MTK_SYS_CIRQ) += cirq/
obj-$(CONFIG_MTK_GIC_EXT) += ext_gic/
obj-$(CONFIG_MTK_FIQ_CACHE) += fiq_cache/
obj-$(CONFIG_MTK_ICE_DEBUG) += ice_debug/
obj-$(CONFIG_MTK_NFC_SUPPORT) += nfc/
obj-$(CONFIG_NFC_CHIP_SUPPORT) += nfc/

obj-$(CONFIG_MMPROFILE) += mmp/
obj-$(CONFIG_MTK_AUXADC)	+= auxadc/
obj-$(CONFIG_MTK_M4U)	+= m4u/
obj-$(CONFIG_MTK_PSEUDO_M4U)   += pseudo_m4u/
obj-$(CONFIG_MTK_GPT_SCHEME_SUPPORT)	+= partition/
obj-$(CONFIG_MTK_SYSENV)	+= sysenv/
obj-$(CONFIG_MTK_PWM)	    += pwm/
obj-$(CONFIG_MTK_TINYSYS_SCP_SUPPORT) += scp/
obj-$(CONFIG_MTK_TINYSYS_SCP_SUPPORT) += audio_ipi/
ifneq ($(CONFIG_MTK_SENSOR_BIO),y)
ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
#obj-$(CONFIG_CUSTOM_KERNEL_SENSORHUB)	+= sensorHub/
obj-$(CONFIG_MTK_HWMON)	    += hwmon/
obj-$(CONFIG_CUSTOM_KERNEL_ALSPS) += alsps/
obj-$(CONFIG_CUSTOM_KERNEL_RGBW) += rgbw/
obj-$(CONFIG_CUSTOM_KERNEL_ACCELEROMETER) += accelerometer/
obj-$(CONFIG_CUSTOM_KERNEL_GYROSCOPE) += gyroscope/
obj-$(CONFIG_CUSTOM_KERNEL_MAGNETOMETER) += magnetometer/
obj-$(CONFIG_CUSTOM_KERNEL_BAROMETER) += barometer/
#obj-$(CONFIG_CUSTOM_KERNEL_HUMIDITY) += humidity/
#obj-$(CONFIG_CUSTOM_KERNEL_STEP_COUNTER)	+= step_counter/
#obj-$(CONFIG_CUSTOM_KERNEL_SIGNIFICANT_MOTION_SENSOR)	+= step_counter/
#obj-$(CONFIG_CUSTOM_KERNEL_ACTIVITY_SENSOR)	+= activity_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_PICK_UP_SENSOR)	+= pick_up_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_SHAKE_SENSOR)	+= shake_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_FACE_DOWN_SENSOR)	+= face_down_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_GESTURE_SENSOR) += gesture/
#obj-$(CONFIG_CUSTOM_KERNEL_PEDOMETER)	+= pedometer/
#obj-$(CONFIG_CUSTOM_KERNEL_HEART)	+= heart_rate_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_TILT_DETECTOR_SENSOR)	+= tilt_detector_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_WAKE_GESTURE_SENSOR)	+= wake_gesture_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_GLANCE_GESTURE_SENSOR)	+= glance_gesture_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_ANSWER_CALL_SENSOR)	+= answer_call_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_GRAVITY_SENSOR)	+= gravity/
#obj-$(CONFIG_CUSTOM_KERNEL_RV_SENSOR)	+= rotationvector/
#obj-$(CONFIG_CUSTOM_KERNEL_LINEARACCEL_SENSOR)	+= linearacceleration/
#obj-$(CONFIG_CUSTOM_KERNEL_GRV_SENSOR)	+= gamerotationvector/
#obj-$(CONFIG_CUSTOM_KERNEL_GMRV_SENSOR)	+= geomagneticrotationvector/
#obj-$(CONFIG_CUSTOM_KERNEL_PDR_SENSOR)	+= pdr_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_FREEFALL_SENSOR)	+= freefall_sensor/
#obj-$(CONFIG_CUSTOM_KERNEL_UNCALI_GYRO_SENSOR)	+= uncali_gyro/
#obj-$(CONFIG_CUSTOM_KERNEL_UNCALI_MAG_SENSOR)	+= uncali_mag/
endif
else
obj-y   += sensor_bio/
endif
obj-$(CONFIG_MTPROF)	+= mtprof/
ifeq ($(CONFIG_MTK_I2C_EXTENSION),y)
#obj-$(CONFIG_MTK_I2C)  += i2c/
endif
obj-$(CONFIG_MTK_LEDS)	+= leds/
obj-$(CONFIG_MTK_VIBRATOR)	+= vibrator/
#obj-$(CONFIG_MTK_SM100)	+= tc1_interface/vib_drv/
obj-$(CONFIG_MTK_ACCDET)	+= accdet/
obj-$(CONFIG_MTK_ECCCI_C2K) += c2k_usb/
obj-y += irtx/
obj-$(CONFIG_MTK_IR_LEARNING_SUPPORT) += ir_learning/
obj-$(CONFIG_MTK_VIDEOCODEC_DRIVER) += videocodec/
obj-$(CONFIG_MTK_JPEG)	+= jpeg/
obj-$(CONFIG_MTK_RTC)	+= rtc/
obj-$(CONFIG_REG_MONITOR_DRIVER) += reg_monitor/
obj-$(CONFIG_MTK_PMIC)	+= power/
ifeq ($(CONFIG_MTK_GAUGE_VERSION),30)
obj-y	+= power/
endif
obj-$(CONFIG_MTK_PMIC_NEW_ARCH)	+= pmic/
obj-$(CONFIG_MTK_PMIC_WRAP_HAL) += pmic_wrap/
obj-$(CONFIG_USB_MTK_HDRC) += usb20/
obj-$(CONFIG_MTK_USBFSH) += usb11/
obj-$(CONFIG_MU3_PHY)	+= mu3phy/
obj-$(CONFIG_USB_MU3D_DRV)	+= mu3d/
obj-$(CONFIG_SSUSB_DRV) += ssusb/
obj-$(CONFIG_USB_XHCI_MTK)	+= xhci/
obj-$(CONFIG_USB_C_SWITCH)	+= usb_c/
obj-$(CONFIG_MTK_GPIO) += gpio/
obj-$(CONFIG_MTK_MT6306_GPIO_SUPPORT) += gpio/
obj-$(CONFIG_SECURITY_SELINUX)	+= selinux_warning/
obj-$(CONFIG_MTK_MEM) += mem/
obj-$(CONFIG_MTK_MEMORY_LOWPOWER) += memory-lowpower/
obj-$(CONFIG_MTK_CMDQ) += cmdq/
obj-$(CONFIG_MTK_BOOT) += boot/
obj-$(CONFIG_MTK_BOOT_REASON) += boot_reason/
obj-$(CONFIG_MTK_CHIP) += chip/
obj-$(CONFIG_MTK_DEVAPC_DRIVER) += devapc/
obj-y += devinfo/
obj-$(CONFIG_CRYPTO_DEV_DX_CC63P) += dxcc/
obj-$(CONFIG_MTK_LCM) += lcm/
obj-$(CONFIG_MTK_LCM) += ext_disp/
obj-$(CONFIG_MTK_HDMI_SUPPORT) += hdmi/
obj-$(CONFIG_MTK_FB) += video/
obj-$(CONFIG_MTK_DYNAMIC_FPS_FRAMEWORK_SUPPORT) += dfrc/
obj-$(CONFIG_MTK_GPU_SUPPORT) += gpu/
obj-$(CONFIG_MTK_SMI_EXT)	+= smi/
obj-$(CONFIG_MTK_CQDMA) += cqdma/
obj-$(CONFIG_MTK_DRAMC) += dramc/
obj-$(CONFIG_MTK_SYNC) += sync/
obj-$(CONFIG_MTK_FREQ_HOPPING) += freqhopping/
#obj-$(CONFIG_MTK_DRAMC) += dramc/
#obj-$(CONFIG_MTK_SYNC) += sync/
obj-$(CONFIG_MTK_FREQ_HOPPING) += freqhopping/
obj-$(CONFIG_MTK_SECURITY_SW_SUPPORT) +=masp/
obj-$(CONFIG_MTK_BLOCK_TAG) +=blocktag/
obj-$(CONFIG_MEDIATEK_SOLUTION)  += sched/
obj-y += cameraisp/src/
obj-$(CONFIG_MTK_IMGSENSOR) += imgsensor/src/
obj-$(CONFIG_MTK_CAM_CAL) += cam_cal/src/
obj-$(CONFIG_MTK_LENS)   += lens/
obj-$(CONFIG_MTK_FLASHLIGHT)	+= flashlight/
obj-$(CONFIG_RT_FLASHLIGHT)	+= flashlight/richtek/
obj-$(CONFIG_MTK_VPU_SUPPORT) += vpu/
obj-$(CONFIG_MTK_CCU)	+= ccu/src/
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/met/),)
obj-$(CONFIG_MTK_MET) += met/
endif
obj-$(CONFIG_MTK_CCCI_DEVICES)	+=  ccci_util/
obj-$(CONFIG_MTK_NET_CCMNI) +=  ccmni/
#obj-$(CONFIG_MTK_ECCCI_DRIVER) += c2k_ccci/
obj-$(CONFIG_MTK_ECCCI_DRIVER) +=  eccci/

#obj-$(CONFIG_MTK_HIBERNATION) += hibcore/

obj-$(CONFIG_MTK_COMBO) += connectivity/
obj-$(CONFIG_MTK_BTIF) += btif/
obj-$(CONFIG_MTK_CONN_MD) += conn_md/
obj-$(CONFIG_MTK_HW_FDE_AES) += fde_aes/

obj-$(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT) += gcpu/
obj-$(CONFIG_TRUSTONIC_TEE_SUPPORT) += gcpu/
ifeq ($(CONFIG_HAVE_MTK_IN_HOUSE_TEE_SUPPORT),y)
#obj-$(CONFIG_TRUSTY) += gcpu/
endif
obj-$(CONFIG_TRUSTONIC_TEE_SUPPORT) += gud/
obj-$(CONFIG_TRUSTONIC_TEE_SUPPORT) += secmem/
obj-$(CONFIG_MICROTRUST_TEE_SUPPORT) += teei/
obj-$(CONFIG_TRUSTKERNEL_TEE_SUPPORT) += tkcore/
obj-$(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT) += mtee/
obj-$(CONFIG_MTK_DEBUG_LATCH)	+= debug_latch/
obj-$(CONFIG_MTK_DEBUG_TRACER)	+= debug_tracer/
obj-$(CONFIG_MTK_DFD_INTERNAL_DUMP) += dfd/
obj-$(CONFIG_MTK_WATCHPOINT) += watchpoint/
obj-$(CONFIG_MTK_SYSTRACKER)   += systracker/
obj-$(CONFIG_MTK_RAM_CONSOLE)	+= ram_console/
obj-$(CONFIG_MTK_DRAM_LOG_STORE)	+= log_store/
obj-$(CONFIG_MTK_USE_RESERVED_EXT_MEM) += exm/
obj-$(CONFIG_RT_REGMAP) += rt-regmap/

obj-$(CONFIG_MTK_SVP) += memory-ssvp/
obj-$(CONFIG_ZONE_MOVABLE_CMA) += zone_movable_cma/

obj-y += performance/

# gator driver is platform-independent
ifneq ($(wildcard $(srctree)/$(src)/gator-driver/),)
obj-$(CONFIG_GATOR) += gator-driver/
endif # gator-driver folder exist

ifeq ($(CONFIG_HAVE_MTK_IN_HOUSE_TEE_SUPPORT),y)
#obj-$(CONFIG_TRUSTY) += mtee/
endif

obj-$(CONFIG_THERMAL)   += thermal/

obj-$(CONFIG_MEDIATEK_SOLUTION)	+= aee/
obj-$(CONFIG_MTK_ATF_LOGGER)	+= atf_log/
obj-$(CONFIG_MTK_MLOG) += mlog/
obj-$(CONFIG_MTK_EMI_BWL)	+= emi_bwl/
obj-$(CONFIG_MTK_MJC_DRIVER)	+= mjc/
#obj-$(CONFIG_MD32_SUPPORT)     += md32/
obj-$(CONFIG_MTK_EMI_MBW) += emi_mbw/
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/tc1_interface/),)
#obj-$(CONFIG_MTK_TC1_FEATURE)   += tc1_interface/
endif
#obj-$(CONFIG_MTK_BTCVSD) += btcvsd/
obj-$(CONFIG_MTK_MTD_NAND) += nand/
obj-$(CONFIG_MTK_DUM_CHAR) += dum-char/
#obj-$(CONFIG_MTK_DYNAMIC_BOOST) += dynamic_boost/

#obj-$(CONFIG_MTK_DISPLAY_120HZ_SUPPORT)	+= rrc/
obj-$(CONFIG_MTK_USB2JTAG_SUPPORT) += usb2jtag/
# Crypto performance tools (disable by default)
obj-n += crypto/

#widevine drm
obj-$(CONFIG_TRUSTONIC_TEE_SUPPORT) += secwidevine/
obj-y += sspm/
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/cpu_stress/),)
obj-$(CONFIG_MTK_CPU_STRESS)	+= cpu_stress/
endif
obj-$(CONFIG_MTK_VOW_SUPPORT)	+= vow/
obj-$(CONFIG_USB)	+= usb_boost/
#obj-$(CONFIG_RT_REGMAP) += rt-regmap/
obj-$(CONFIG_MEDIATEK_SOLUTION)	+= cmo/
obj-$(CONFIG_MTK_SMARTPA_SOUND)	+= smartpa/
obj-$(CONFIG_PCIE_MTK_UNIT)	+= pcie_ut/
obj-y += mtprintk/
obj-$(CONFIG_MTK_ICCS_SUPPORT)+= iccs/
obj-$(CONFIG_MACH_MT8167) += xo/
obj-$(CONFIG_MTK_MD_DIRECT_TETHERING_SUPPORT)	+= pkt_track/
obj-$(CONFIG_MTK_GZ_KREE)  += geniezone/

# Vanzo:yucheng on: Wed, 12 Nov 2014 21:09:12 +0800
# added for aosp management to define some macros

ifneq ($(strip $(project_name)),)
MTK_INTERNAL_CDEFS := $(foreach t,$(AUTO_ADD_GLOBAL_DEFINE_BY_NAME),$(if $(filter-out no NO none NONE false FALSE,$($(t))),-D$(t)))
MTK_INTERNAL_CDEFS += $(foreach t,$(AUTO_ADD_GLOBAL_DEFINE_BY_VALUE),$(if $(filter-out no NO none NONE false FALSE,$($(t))),$(foreach v,$(shell echo $($(t)) | tr '[a-z]' '[A-Z]'),-D$(v))))
MTK_INTERNAL_CDEFS += $(foreach t,$(AUTO_ADD_GLOBAL_DEFINE_BY_NAME_VALUE),$(if $(filter-out no NO none NONE false FALSE,$($(t))),-D$(t)=\"$($(t))\"))
VANZO_INTERNAL_CDEFS := $(foreach t,$(VANZO_DEFINE_BY_NAME),$(if $(filter-out no NO none NONE false FALSE,$($(t))),-D$(t)))
VANZO_INTERNAL_CDEFS += $(foreach t,$(VANZO_DEFINE_BY_VALUE),$(if $(filter-out no NO none NONE false FALSE,$($(t))),$(foreach v,$(shell echo $($(t)) | tr '[a-z]' '[A-Z]'),-D$(v))))
VANZO_INTERNAL_CDEFS += $(foreach t,$(VANZO_DEFINE_BY_NAME_VALUE),$(if $(filter-out no NO none NONE false FALSE,$($(t))),-D$(t)=\"$($(t))\"))


subdir-ccflags-y += $(VANZO_INTERNAL_CDEFS)
subdir-ccflags-y += $(MTK_INTERNAL_CDEFS)
endif
# End of Vanzo:yucheng
# Vanzo:yangbinbin on: Mon, 24 Oct 2016 16:40:39 +0800
# add hall
ifneq ($(strip $(CUSTOM_KERNEL_HALL)),)
    obj-y += hall/
endif
# End of Vanzo: yangbinbin
