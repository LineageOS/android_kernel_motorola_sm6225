DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(AW96XX_USB_USE_ONLINE),true)
	KERNEL_CFLAGS += CONFIG_AW96XX_POWER_SUPPLY_ONLINE=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := awinic_sar.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/sensors_class.ko
#LOCAL_REQUIRED_MODULES := sensors_class.ko
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

