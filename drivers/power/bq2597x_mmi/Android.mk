DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := bq2597x_mmi.ko

ifeq ($(BQ2597X_DUAL_ENABLE),true)
	KERNEL_CFLAGS += CONFIG_BQ2597X_DUAL=y
endif

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif


include $(DLKM_DIR)/AndroidKernelModule.mk
