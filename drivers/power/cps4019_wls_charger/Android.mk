DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := cps4019_wls_charger.ko


ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif


include $(DLKM_DIR)/AndroidKernelModule.mk
