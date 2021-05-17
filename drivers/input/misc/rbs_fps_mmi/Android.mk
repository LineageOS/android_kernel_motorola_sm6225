DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
ETS_FPS_MMI_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
ETS_FPS_MMI_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif


include $(CLEAR_VARS)
LOCAL_MODULE := rbs_fps_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(ETS_FPS_MMI_MODULE_PATH)
include $(DLKM_DIR)/AndroidKernelModule.mk

