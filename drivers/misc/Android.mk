DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := cirrus_cs40l20.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

# include $(CLEAR_VARS)
# LOCAL_MODULE := cs40l20-tables.ko
# LOCAL_MODULE_TAGS := optional
# LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
# include $(DLKM_DIR)/AndroidKernelModule.mk
