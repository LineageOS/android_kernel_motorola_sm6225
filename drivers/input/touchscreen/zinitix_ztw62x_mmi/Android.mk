DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := zinitix_ztw62x_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)

ifneq ($(findstring touchscreen_mmi.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
    KBUILD_OPTIONS += CONFIG_INPUT_TOUCHSCREEN_MMI=y
    LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/touchscreen_mmi.ko
endif
include $(DLKM_DIR)/AndroidKernelModule.mk


KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
