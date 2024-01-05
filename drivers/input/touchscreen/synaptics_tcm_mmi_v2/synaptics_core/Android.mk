DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DRM_PANEL_NOTIFICATIONS),true)
    KERNEL_CFLAGS += CONFIG_DRM_PANEL_NOTIFICATIONS=y
endif

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
SYNA_TCM_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
SYNA_TCM_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_core_module.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(SYNA_TCM_MODULE_PATH)
ifneq ($(findstring touchscreen_mmi.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
    KERNEL_CFLAGS += CONFIG_INPUT_TOUCHSCREEN_MMI=y
    LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/touchscreen_mmi.ko
endif
ifneq ($(findstring synaptics_i2c.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
    LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/synaptics_i2c.ko
else
    LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/synaptics_spi.ko
endif

KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk
