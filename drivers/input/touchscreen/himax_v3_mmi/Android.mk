DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
HIMAX_MMI_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
HIMAX_MMI_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

ifeq ($(HIMAX_V3_MMI_HX83102),true)
	KERNEL_CFLAGS += CONFIG_INPUT_HIMAX_V3_MMI_IC_NAME=hx83102
endif

include $(CLEAR_VARS)
LOCAL_MODULE := himax_v3_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(HIMAX_MMI_MODULE_PATH)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(call first-makefiles-under,$(LOCAL_PATH))
