#
# Makefile for the linux pstorefs routines.
#

obj-y += pstore.o

pstore-objs += inode.o platform.o
obj-$(CONFIG_PSTORE_FTRACE)	+= ftrace.o

ramoops-objs += ram.o ram_core.o ram_device.o
obj-$(CONFIG_PSTORE_RAM)	+= ramoops.o

pstore_zone-objs += zone.o
obj-$(CONFIG_PSTORE_ZONE)	+= pstore_zone.o

pstore_blk-objs += blk.o
obj-$(CONFIG_PSTORE_BLK)	+= pstore_blk.o
