DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
ifneq ($(findstring goodix_brl_i2c.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
    KERNEL_CFLAGS += CONFIG_USE_TOUCHSCREEN_MMI_I2C=y
    LOCAL_MODULE := goodix_brl_i2c.ko
else
    LOCAL_MODULE := goodix_brl_spi.ko
endif
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)

include $(DLKM_DIR)/AndroidKernelModule.mk
