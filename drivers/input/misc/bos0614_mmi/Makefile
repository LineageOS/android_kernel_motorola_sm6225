obj-m := bosDriverModule.o
bosDriverModule-y := i2cLinux.o
bosDriverModule-y += bosDriver.o
bosDriverModule-y += libs/dk-core/src/bsp/drivers/haptic/bos0614Driver.o
bosDriverModule-y += libs/dk-core/contribs/comm-stack-dk/data.o

ccflags-y += -I$(src)/libs/dk-core/src
ccflags-y += -I$(src)/libs/dk-core/
ccflags-y += -std=gnu99
ccflags-y += -DDEBUG
ccflags-y += -DLITTLE_ENDIAN=1234
ccflags-y += -DBYTE_ORDER=LITTLE_ENDIAN

PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)
