DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := moto_mmap_fault.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)

ifeq ($(TUNE_MMAP_READAROUND), true)
	KERNEL_CFLAGS += KCFLAGS=-DTUNE_MMAP_READAROUND
	KBUILD_OPTIONS += KCFLAGS=-DTUNE_MMAP_READAROUND
endif

include $(DLKM_DIR)/AndroidKernelModule.mk
