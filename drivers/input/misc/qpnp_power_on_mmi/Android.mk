DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := qpnp-power-on-mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
KBUILD_OPTIONS_GKI += MODULE_KERNEL_VERSION=$(TARGET_KERNEL_VERSION)
include $(DLKM_DIR)/AndroidKernelModule.mk

