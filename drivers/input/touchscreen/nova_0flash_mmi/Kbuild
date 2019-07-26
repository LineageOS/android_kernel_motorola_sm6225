# add -Wall to try to catch everything we can.
EXTRA_CFLAGS += -Wall
EXTRA_CFLAGS += -I$(TOP)/motorola/kernel/modules/include \
		-I$(TOP)/motorola/kernel/modules/drivers/input/touchscreen/nova_0flash_mmi

ifeq ($(filter m y, $(CONFIG_TOUCHSCREEN_NT36xxx_HOSTDL_SPI)),)
	obj-m := nova_0flash_mmi.o
endif

nova_0flash_mmi-objs += nt36xxx.o
nova_0flash_mmi-objs += nt36xxx_ext_proc.o
nova_0flash_mmi-objs += nt36xxx_fw_update.o
nova_0flash_mmi-objs += nt36xxx_mp_ctrlram.o
