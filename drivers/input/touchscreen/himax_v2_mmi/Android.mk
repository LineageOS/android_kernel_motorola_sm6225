DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
HIMAX_MMI_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
HIMAX_MMI_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

ifeq ($(HIMAX_V2_MMI_0FLASH),true)
	KERNEL_CFLAGS += CONFIG_INPUT_TOUCHSCREEN_HIMAX_V2_MMI_0FLASH=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := himax_v2_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(HIMAX_MMI_MODULE_PATH)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(call first-makefiles-under,$(LOCAL_PATH))
