DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_dsx_fw_update.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

#include $(CLEAR_VARS)
#LOCAL_MODULE := synaptics_dsx_rmi_dev.ko
#LOCAL_MODULE_TAGS := optional
#LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
#include $(DLKM_DIR)/AndroidKernelModule.mk

#This requires F54 support, thus disable it for now
#include $(CLEAR_VARS)
#LOCAL_MODULE := synaptics_dsx_test_reporting.ko
#LOCAL_MODULE_TAGS := optional
#LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
#include $(DLKM_DIR)/AndroidKernelModule.mk
