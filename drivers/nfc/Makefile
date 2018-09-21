#
# Makefile for nfc devices
#

#obj-$(CONFIG_NFC_PN553_DEVICES)		+= pn553.o
obj-$(CONFIG_LGE_NFC_PN553)             += pn553_lge.o
obj-$(CONFIG_LGE_NFC_PN553)	+= pn553_lge_hwadapter.o
obj-$(CONFIG_CXD224X_NFC)  += cxd224x-i2c.o
ccflags-$(CONFIG_NFC_PN553_DEVICES) := -DDEBUG
obj-$(CONFIG_LGE_NFC_PN547_C2)	:= pn547_lge.o
obj-$(CONFIG_LGE_NFC_PN547_C2)	+= pn547_lge_hwadapter.o
