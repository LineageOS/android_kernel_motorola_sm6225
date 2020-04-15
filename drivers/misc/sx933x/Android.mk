DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := sx933x_sar.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

ifeq ($(SX933X_USB_CAL),true)
	KERNEL_CFLAGS += CONFIG_SX933X_USB_CAL=y
endif

ifeq ($(SX933X_FLIP_CAL),true)
	KERNEL_CFLAGS += CONFIG_SX933X_FLIP_CAL=y
endif
