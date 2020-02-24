DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := mmi_charger.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(call first-makefiles-under,$(LOCAL_PATH))
