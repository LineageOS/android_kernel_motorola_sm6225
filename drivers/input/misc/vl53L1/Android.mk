DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(PRODUCT_USE_TOF_vl53l3),true)
	KERNEL_CFLAGS += CONFIG_INPUT_MISC_TOF_USE_VL53L3=y
endif

ifeq ($(PRODUCT_USE_PMIC_WL2864C),true)
	KERNEL_CFLAGS += CONFIG_INPUT_MISC_PMIC_WL2864C=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := stmvl53l1.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

