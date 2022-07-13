DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := aw_haptic_hv.ko
LOCAL_MODULE_TAGS := optional

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

ifeq ($(PRODUCT_HAS_AWINIC_HAPTIC),true)
	KERNEL_CFLAGS += CONFIG_INPUT_AWINIC_HAPTIC=y
endif

include $(DLKM_DIR)/AndroidKernelModule.mk
