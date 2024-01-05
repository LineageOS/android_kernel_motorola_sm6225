DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(MMI_QC3P_TURBO_CHARGER),true)
	KERNEL_CFLAGS += CONFIG_MMI_QC3P_TURBO_CHARGER_ISP_5G=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := mmi_qc3p_wt6670f.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
