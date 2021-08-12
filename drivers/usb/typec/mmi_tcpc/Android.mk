DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

#ifeq ($(TCPC_RT1711H),true)
	KERNEL_CFLAGS += CONFIG_TCPC_RT1711H=y
#endif

#ifeq ($(TCPC_CLASS),true)
	KERNEL_CFLAGS += CONFIG_TCPC_CLASS=y
#endif

#ifeq ($(USB_POWER_DELIVERY),true)
	KERNEL_CFLAGS += CONFIG_USB_POWER_DELIVERY=y
#endif

include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := tcpc_rt1711h.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk


include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := tcpc_class.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk


include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := rt_pd_manager.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
