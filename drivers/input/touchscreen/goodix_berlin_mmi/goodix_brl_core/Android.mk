DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := goodix_brl_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
ifneq ($(findstring goodix_brl_i2c.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
    LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/goodix_brl_i2c.ko
else
    LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/goodix_brl_spi.ko
endif
include $(DLKM_DIR)/AndroidKernelModule.mk
