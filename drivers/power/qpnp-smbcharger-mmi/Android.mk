DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(MOTO_PD_CHARGER),true)
	KERNEL_CFLAGS += CONFIG_PD_CHARGER_MMI=y
endif

ifeq ($(BQ2597X_DUAL_ENABLE),true)
        KERNEL_CFLAGS += CONFIG_BQ2597X_DUAL=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := qpnp-smbcharger-mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

