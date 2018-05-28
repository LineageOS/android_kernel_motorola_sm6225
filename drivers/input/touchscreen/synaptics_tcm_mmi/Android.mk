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
 FIRMWARE_FILES := $(shell cd $(LOCAL_PATH) ; find -L -maxdepth 1 -name "*.img")

# Destination folder in O is different
ifeq ($(call is-platform-version-at-least,O),true)
FIRMWARE_DEST_PATH = $(TARGET_OUT_VENDOR)/firmware/synaptics
else
FIRMWARE_DEST_PATH = $(TARGET_OUT_ETC)/firmware/synaptics
endif

define install-firmware-file
$(eval $(install-to-destination))
endef

define install-to-destination
     include $$(CLEAR_VARS)
     LOCAL_MODULE_TAGS := optional
     LOCAL_MODULE := $(1)
     LOCAL_MODULE_PATH := $$(FIRMWARE_DEST_PATH)
     LOCAL_MODULE_CLASS := ETC
     LOCAL_SRC_FILES := $$(LOCAL_MODULE)
     include $$(BUILD_PREBUILT)
endef

 $(foreach f,$(FIRMWARE_FILES),$(call install-firmware-file,$(f)))

include $(CLEAR_VARS)
LOCAL_MODULE := synaptics_tcm_reflash.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_REQUIRED_MODULES := $(FIRMWARE_FILES)
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

