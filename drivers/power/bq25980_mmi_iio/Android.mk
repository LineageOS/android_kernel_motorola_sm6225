DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := bq25980_mmi_iio.ko

ifeq ($(MMI_BQ25980_BULK_READ),true)
        KERNEL_CFLAGS += CONFIG_MMI_BQ25980_BULK_READ=y
endif

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif


include $(DLKM_DIR)/AndroidKernelModule.mk
