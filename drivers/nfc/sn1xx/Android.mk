DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := nfc_i2c.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/mmi_info.ko
include $(DLKM_DIR)/AndroidKernelModule.mk
