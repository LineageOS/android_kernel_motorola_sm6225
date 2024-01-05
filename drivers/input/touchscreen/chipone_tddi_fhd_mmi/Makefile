KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(shell pwd) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(shell pwd) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

obj-$(CONFIG_TOUCHSCREEN_CHIPONE) += chipone-tddi.o

chipone-tddi-objs := cts_i2c_driver.o
chipone-tddi-objs += cts_core.o cts_sfctrlv2.o cts_spi_flash.o cts_firmware.o cts_test.o
chipone-tddi-objs += cts_charger_detect.o
chipone-tddi-objs += cts_hostcomm.o
chipone-tddi-objs += cts_tcs.o
chipone-tddi-objs += cts_platform.o
chipone-tddi-objs += cts_tool.o
chipone-tddi-objs += cts_sysfs.o
chipone-tddi-objs += cts_strerror.o

