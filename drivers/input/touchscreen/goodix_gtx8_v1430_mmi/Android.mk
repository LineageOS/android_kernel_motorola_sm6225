DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifneq ($(findstring touchscreen_mmi.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
	KERNEL_CFLAGS += CONFIG_INPUT_TOUCHSCREEN_MMI=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := goodix_v1430_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := goodix_v1430_update_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE := goodix_v1430_ts_tools_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(call first-makefiles-under,$(LOCAL_PATH))
