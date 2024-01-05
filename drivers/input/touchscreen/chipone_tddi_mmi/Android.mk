DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
CHIPONE_MMI_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
CHIPONE_MMI_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif
ifeq ($(CHIPONE_ESD),true)
	KERNEL_CFLAGS += CONFIG_CHIPONE_ESD=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := chipone_tddi_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(CHIPONE_MMI_MODULE_PATH)
include $(DLKM_DIR)/AndroidKernelModule.mk
