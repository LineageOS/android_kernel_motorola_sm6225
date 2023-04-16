all: modules

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

KBUILD_EXTRA_SYMBOLS += $(M)/../../../sensors/Module.symvers
KBUILD_EXTRA_SYMBOLS += $(M)/../../../mmi_relay/Module.symvers
KBUILD_EXTRA_SYMBOLS += $(OUT_DIR)/../../vendor/qcom/opensource/display-drivers/msm/Module.symvers
