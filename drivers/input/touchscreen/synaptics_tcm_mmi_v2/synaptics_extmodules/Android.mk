DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifneq ($(findstring touchscreen_mmi.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
	KERNEL_CFLAGS += CONFIG_INPUT_TOUCHSCREEN_MMI=y
endif
ifeq ($(DRM_PANEL_NOTIFICATIONS),true)
	KERNEL_CFLAGS += CONFIG_DRM_PANEL_NOTIFICATIONS=y
endif

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
SYNA_TCM_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
SYNA_TCM_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_device.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(SYNA_TCM_MODULE_PATH)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/synaptics_core_module.ko
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_diagnostics.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(SYNA_TCM_MODULE_PATH)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/synaptics_core_module.ko
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_recovery.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(SYNA_TCM_MODULE_PATH)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/synaptics_core_module.ko
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_reflash.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(SYNA_TCM_MODULE_PATH)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/synaptics_core_module.ko
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_testing.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(SYNA_TCM_MODULE_PATH)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/synaptics_core_module.ko
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_zeroflash.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(SYNA_TCM_MODULE_PATH)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/synaptics_core_module.ko
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

