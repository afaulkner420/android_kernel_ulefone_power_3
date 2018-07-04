
ifneq ($(strip $(M)),)
	obj-m += tkcore.o
else
	obj-y += tkcore.o
endif

M ?=$(srctree)/drivers/misc/mediatek/tkcore

#ccflags-y+=-Werror
ccflags-y+=-I$(M)/include/linux
ccflags-y+=-I$(M)/include

ccflags-y+=-I$(M)

tkcore-objs:=   \
		tee_core.o  \
		tee_context.o  \
		tee_session.o  \
		tee_shm.o  \
		tee_supp_com.o  \
		tee_sysfs.o \
		tee_kernel_api.o \
		tee_wait_queue.o \
		tee_tui.o \
		tee_tui_hal.o \
		tee_procfs.o  \
		tee_ta_mgmt.o \
		pm.o

ifeq ($(CONFIG_TRUSTKERNEL_TEE_FP_SUPPORT),y)
	tkcore-objs += tee_fp.o
	ccflags-y += -DTKCORE_FP_SUPPORT
	ccflags-y += -I$(M)/core
endif
