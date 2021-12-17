DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := sgm4154x_charger.ko

ifeq ($(MMI_QC3P_TURBO_CHARGER),true)
        KERNEL_CFLAGS += CONFIG_MMI_QC3P_TURBO_CHARGER=y
endif

ifeq ($(MMI_HAWAO_VBUS_SAMPLE_RATIO),true)
        KERNEL_CFLAGS += CONFIG_MMI_HAWAO_VBUS_SAMPLE_RATIO=y
endif

LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)

LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/mmi_discrete_charger_class.ko
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki

include $(DLKM_DIR)/AndroidKernelModule.mk
