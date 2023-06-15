LOCAL_PATH:= $(call my-dir)

ifneq ($(findstring stmvl53l1.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
VL53L1_BARE_DRV_INC += motorola/kernel/modules/drivers/input/misc/vl53L1
VL53L1_BARE_DRV_INC += motorola/kernel/modules/drivers/input/misc/vl53L1/inc
else ifneq ($(findstring stmvl53l3.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
VL53L1_BARE_DRV_INC += motorola/kernel/modules/drivers/input/misc/vl53L1_14_1_2
VL53L1_BARE_DRV_INC += motorola/kernel/modules/drivers/input/misc/vl53L1_14_1_2/inc
else
VL53L1_BARE_DRV_INC += kernel/msm-4.14/drivers/input/misc/vl53L1
VL53L1_BARE_DRV_INC += kernel/msm-4.14/drivers/input/misc/vl53L1/inc
endif

include $(CLEAR_VARS)
LOCAL_MODULE := vl53l1_reg
LOCAL_PROPRIETARY_MODULE := true
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_C_INCLUDES += $(VL53L1_BARE_DRV_INC)
LOCAL_SRC_FILES := vl53l1_reg.c
LOCAL_SHARED_LIBRARIES:=libc
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := vl53l1_iotest
LOCAL_PROPRIETARY_MODULE := true
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_C_INCLUDES += $(VL53L1_BARE_DRV_INC)
LOCAL_SRC_FILES := vl53l1_iotest.c
LOCAL_SHARED_LIBRARIES:=libc
include $(BUILD_EXECUTABLE)

ifneq ($(findstring stmvl53l3.ko,$(BOARD_VENDOR_KERNEL_MODULES)),)
include $(CLEAR_VARS)
LOCAL_MODULE := phio
LOCAL_PROPRIETARY_MODULE := true
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_C_INCLUDES += $(VL53L1_BARE_DRV_INC)
LOCAL_SRC_FILES := phio_14_1_2.c
LOCAL_SHARED_LIBRARIES := libc
include $(BUILD_EXECUTABLE)
else
include $(CLEAR_VARS)
LOCAL_MODULE := phio
LOCAL_PROPRIETARY_MODULE := true
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_C_INCLUDES += $(VL53L1_BARE_DRV_INC)
LOCAL_SRC_FILES := phio.c
LOCAL_SHARED_LIBRARIES := libc
include $(BUILD_EXECUTABLE)
endif
