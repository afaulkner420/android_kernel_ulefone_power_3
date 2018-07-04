#
# Makefile for the fingerprint drivers.
#

# Each configuration option enables a list of files.
EXTRA_CFLAGS +=-Wno-error=date-time
EXTRA_CFLAGS +=-Wno-error=declaration-after-statement
EXTRA_CFLAGS +=-Wno-error

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include \
					-I$(srctree)/drivers/misc/mediatek/include/mt-plat \
					-I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)

obj-y += sunwavecorp.o
obj-y += platform.o
obj-y += misce.o
obj-y += mt6763_dts.o
obj-y += mt6763_dts_plat.o
