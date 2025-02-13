#
# Makefile for the Synaptics TCM touchscreen driver.
#

# Each configuration option enables a list of files.

TCM_CORE=tcm/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/synaptics_tcm2/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/synaptics_tcm2/tcm/

synaptics_tcm2-y += syna_tcm2.o
synaptics_tcm2-y += \
					$(TCM_CORE)synaptics_touchcom_core_v1.o \
					$(TCM_CORE)synaptics_touchcom_core_v2.o \
 					$(TCM_CORE)synaptics_touchcom_func_base.o \
 					$(TCM_CORE)synaptics_touchcom_func_touch.o \
					$(TCM_CORE)synaptics_touchcom_func_reflash.o \
					$(TCM_CORE)synaptics_touchcom_func_romboot.o

# synaptics_tcm2-y += syna_tcm2_testing.o
# synaptics_tcm2-y += syna_tcm2_sysfs.o
# synaptics_tcm2-y += syna_tcm2_platform_spi.o

obj-$(CONFIG_TOUCHSCREEN_SYNA_TCM2) += synaptics_tcm2.o
