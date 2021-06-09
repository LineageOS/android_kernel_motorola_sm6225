DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := watchdog_cpu_ctx.ko
LOCAL_MODULE_TAGS := optional
ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/mmi_info.ko
include $(DLKM_DIR)/AndroidKernelModule.mk
