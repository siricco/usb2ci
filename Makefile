wintv-usb2ci-objs := wintv-ci-ca.o wintv-ci-ci.o wintv-ci-en50221.o wintv-ci-pcmcia.o wintv-ci-core.o

obj-m += wintv-usb2ci.o

ccflags-y += -Wall -Idrivers/media/dvb-core/
