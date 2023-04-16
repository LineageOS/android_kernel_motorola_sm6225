DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := mcDrvModule.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
ifeq ($(RSU_SELECT_INTERNAL_CLOCK), true)
KERNEL_CFLAGS += RSU_INTERNAL_CLOCK=y
endif
include $(DLKM_DIR)/AndroidKernelModule.mk
