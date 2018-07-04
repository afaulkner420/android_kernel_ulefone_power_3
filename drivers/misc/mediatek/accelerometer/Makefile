# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
# Vanzo:yucheng on: Wed, 12 Nov 2014 21:08:36 +0800
# added for aosp management to import our variable
project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))
ifneq ($(strip $(project_name)),)
-include $(srctree)/../zprojects/$(project_name)/$(project_name).mk
VANZO_PROJECT_HEADERS:=$(srctree)/../zprojects/$(project_name)/kernel-headers
ccflags-y += -I$(VANZO_PROJECT_HEADERS)
endif
# End of Vanzo:yucheng

obj-y   +=  accel.o accel_factory.o

# Vanzo:yucheng on: Wed, 12 Nov 2014 21:06:42 +0800
project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))

ifneq ($(project_name),)
custom_kernel_accelerometer:= $(addsuffix /, $(shell echo $(CUSTOM_KERNEL_ACCELEROMETER)))
ifneq ($(custom_kernel_accelerometer),)
obj-y += $(custom_kernel_accelerometer)
endif
else
# End of Vanzo:yucheng

ifeq ($(CONFIG_MTK_SCP_SENSORHUB_V1),y)
obj-$(CONFIG_MTK_ACCELHUB)		+=	accelhub/
else
obj-$(CONFIG_MTK_BMA2XX)   +=  bma2xx/
obj-$(CONFIG_MTK_MC3433)   +=  mc3433/
obj-$(CONFIG_MTK_LSM6DS3HG)   +=  lsm6ds3hg/
obj-$(CONFIG_MTK_BMI160_SPI)   +=  bmi160-spi/
obj-$(CONFIG_MTK_BMI160_I2C)   +=  bmi160-i2c/
obj-$(CONFIG_MTK_ICM206XX_A)   +=  icm206xx_a/
endif
# Vanzo:yucheng on: Tue, 06 Jan 2015 10:34:15 +0800
# added for new structure config alsps
endif
# End of Vanzo:yucheng