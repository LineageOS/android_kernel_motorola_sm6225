DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_spi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk


include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_i2c.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk


include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_core.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_device.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_diagnostics.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_recovery.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_reflash.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_testing.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_touch.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_zeroflash.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

