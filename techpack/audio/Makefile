# auto-detect subdirs
ifeq ($(CONFIG_ARCH_SDXPOORWILLS), y)
include $(srctree)/techpack/audio/config/sdxpoorwillsauto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdxpoorwillsauto.conf)
endif
ifeq ($(CONFIG_ARCH_SM8150), y)
include $(srctree)/techpack/audio/config/sm8150auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sm8150auto.conf)
endif
ifeq ($(CONFIG_ARCH_SDMSHRIKE), y)
include $(srctree)/techpack/audio/config/sm8150auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sm8150auto.conf)
endif
ifeq ($(CONFIG_ARCH_KONA), y)
include $(srctree)/techpack/audio/config/konaauto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/konaauto.conf)
endif
ifeq ($(CONFIG_ARCH_LITO), y)
include $(srctree)/techpack/audio/config/litoauto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/litoauto.conf)
endif
ifeq ($(CONFIG_ARCH_BENGAL), y)
include $(srctree)/techpack/audio/config/bengalauto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/bengalauto.conf)
endif
ifeq ($(CONFIG_ARCH_SDM660), y)
include $(srctree)/techpack/audio/config/sdm660auto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/sdm660auto.conf)
endif
ifneq (,$(filter y, $(CONFIG_BATHENA_DTB) $(CONFIG_GUAM_DTB) $(CONFIG_GUAMNA_DTB) $(CONFIG_GUAMP_DTB)))
include $(srctree)/techpack/audio/config/guamauto.conf
export $(shell sed 's/=.*//' $(srctree)/techpack/audio/config/guamauto.conf)
endif

# Use USERINCLUDE when you must reference the UAPI directories only.
USERINCLUDE     += \
                -I$(srctree)/techpack/audio/include/uapi \
                -I$(srctree)/techpack/audio/include

# Use LINUXINCLUDE when you must reference the include/ directory.
# Needed to be compatible with the O= option
LINUXINCLUDE    += \
                -I$(srctree)/techpack/audio/include/uapi \
                -I$(srctree)/techpack/audio/include

ifeq ($(CONFIG_ARCH_SDXPOORWILLS), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdxpoorwillsautoconf.h
endif
ifeq ($(CONFIG_ARCH_SM8150), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sm8150autoconf.h
endif
ifeq ($(CONFIG_ARCH_SDMSHRIKE), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sm8150autoconf.h
endif
ifeq ($(CONFIG_ARCH_KONA), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/konaautoconf.h
endif
ifeq ($(CONFIG_ARCH_LITO), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/litoautoconf.h
endif
ifeq ($(CONFIG_ARCH_BENGAL), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/bengalautoconf.h
endif
ifeq ($(CONFIG_ARCH_SDM660), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdm660autoconf.h
endif
ifeq ($(CONFIG_DEVON_DTB), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/devonautoconf.h
endif
ifneq (,$(filter y, $(CONFIG_BATHENA_DTB) $(CONFIG_GUAM_DTB) $(CONFIG_GUAMNA_DTB) $(CONFIG_GUAMP_DTB)))
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/guamautoconf.h
endif
ifeq ($(CONFIG_HAWAO_DTB), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/hawaoautoconf.h
endif
ifeq ($(CONFIG_RHODE_DTB), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/rhodeautoconf.h
endif

obj-y += soc/
obj-y += dsp/
obj-y += ipc/
obj-y += asoc/
