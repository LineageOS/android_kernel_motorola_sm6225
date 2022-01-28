obj-$(CONFIG_BQ2597X_MMI) += bq2597x_mmi.o

ifneq ($(filter m y,$(CONFIG_BQ2597X_DUAL)),)
	bq2597x_mmi-objs += bq25980_charger.o
else
	bq2597x_mmi-objs += bq2597x_single_mmi.o
endif
