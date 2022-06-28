#
# Makefile for the fingerprint device.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_GOODIX_CAP_FINGERPRINT) := fpc1022_tee.o
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/teei/400/tz_driver/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/teei/400/common/include/
