DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(VIBRATOR_NOISE_CAMERA), true)
	KBUILD_OPTIONS += CONFIG_VIBRATOR_NOISE_CAMERA=y
endif

KBUILD_OPTIONS += MODULE_KERNEL_VERSION=$(TARGET_KERNEL_VERSION)

include $(CLEAR_VARS)
LOCAL_MODULE := ldo_vibrator_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

