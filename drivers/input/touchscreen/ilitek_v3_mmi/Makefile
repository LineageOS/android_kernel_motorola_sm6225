KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(shell pwd) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(shell pwd) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

KBUILD_EXTRA_SYMBOLS += $(M)/../../../mmi_info/$(GKI_OBJ_MODULE_DIR)/Module.symvers
KBUILD_EXTRA_SYMBOLS += $(M)/../touchscreen_mmi/$(GKI_OBJ_MODULE_DIR)/Module.symvers
