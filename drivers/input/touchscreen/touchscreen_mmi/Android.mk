DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DRM_DYNAMIC_REFRESH_RATE),true)
	KERNEL_CFLAGS += CONFIG_DRM_DYNAMIC_REFRESH_RATE=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := touchscreen_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
