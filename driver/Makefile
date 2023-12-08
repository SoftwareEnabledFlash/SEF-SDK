# if KERNELRELEASE is defined, we've been invoked from the
# # kernel build system and can use its language.
obj-m := sef.o
sef-objs := sef_core.o sef_aen.o sef_sys.o
ifneq (${SEFSDKVersion},)
ccflags-y := -DSEFSDKVersion="\"${SEFSDKVersion}\""
endif
ifneq (${KERNELRELEASE},)
# Otherwise we were called directly from the command line.
# Invoke the kernel build system.
else
KERNEL_SOURCE := ${SEF_KERNEL_SRC}
PWD := $(shell pwd)
endif
default:
	${MAKE} -C ${KERNEL_SOURCE} M=${PWD} modules

modules_install:
	${MAKE} -C ${KERNEL_SOURCE} M=${PWD} modules_install
	@mkdir -p /etc/modules-load.d/
	$(shell echo "sef" > /etc/modules-load.d/sef.conf)

clean:
	${MAKE} -C ${KERNEL_SOURCE} M=${PWD} clean
