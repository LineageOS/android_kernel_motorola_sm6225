# add -Wall to try to catch everything we can.
EXTRA_CFLAGS += -Wall
EXTRA_CFLAGS += -I$(ANDROID_BUILD_TOP)/motorola/kernel/modules/include \
		-I$(ANDROID_BUILD_TOP)/motorola/kernel/modules/drivers/input/touchscreen/chipone_tddi_mmi
ifneq ($(filter m y,$(CONFIG_CHIPONE_ESD)),)
EXTRA_CFLAGS += -DCONFIG_CTS_ESD_PROTECTION
endif

obj-m := chipone_tddi_mmi.o

chipone_tddi_mmi-objs += cts_i2c_driver.o
chipone_tddi_mmi-objs += cts_core.o
chipone_tddi_mmi-objs += cts_platform.o
chipone_tddi_mmi-objs += cts_sfctrlv2.o
chipone_tddi_mmi-objs += cts_spi_flash.o
chipone_tddi_mmi-objs += cts_firmware.o
chipone_tddi_mmi-objs += cts_tool.o
chipone_tddi_mmi-objs += cts_sysfs.o
chipone_tddi_mmi-objs += cts_test.o
chipone_tddi_mmi-objs += cts_charger_detect.o
