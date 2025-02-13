#
# Makefile for the Synaptics TCM touchscreen driver.
#

# Each configuration option enables a list of files.

NAME=

TCM_CORE=tcm/
TCM_DRIVER_SRC=drivers/input/touchscreen/synaptics_tcm2/

obj-y += syna_tcm2.o
obj-y += syna_tcm2_testing.o
obj-y += syna_tcm2_sysfs.o
obj-y += syna_tcm2_platform_spi.o
obj-y += $(TCM_CORE)synaptics_touchcom_func_reflash.o
obj-y += $(TCM_CORE)synaptics_touchcom_func_romboot.o
obj-y += $(TCM_CORE)synaptics_touchcom_func_base.o
obj-y += $(TCM_CORE)synaptics_touchcom_func_touch.o
obj-y += $(TCM_CORE)synaptics_touchcom_core_v2.o
obj-y += $(TCM_CORE)synaptics_touchcom_core_v1.o

ccflags-y += -I$(srctree)/$(TCM_DRIVER_SRC)
ccflags-y += -I$(srctree)/$(TCM_DRIVER_SRC)$(TCM_CORE)
