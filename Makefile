ifndef KERNELDIR
KERNELDIR  := /lib/modules/$(shell uname -r)/build
endif

ifndef MODDIR
MODDIR := $(PWD)
endif

obj-m := unicorn_usb_eth.o

CXX = g++
DRIVER_VERSION=\"USB-ADL-7-2.0-0.2.0.11\"
EXTRA_CFLAGS := -DDRIVER_VERSION=$(DRIVER_VERSION) -DPKG_VERSION=$(PKG_VERSION) -D_USB_DRIVER -DDEBUG=1
CXXFLAGS = -mregparm=3 -fno-rtti -fno-exceptions $(CFLAGS) $(EXTRA_CFLAGS)
OBJS = src/unicorn_usbdrv.o src/interruptmonitor.o src/usb_protocolcreator.o src/accessmechanism.o src/C-interface.o src/linrapi.o src/msw.o ./src/crc.o src/amas.o src/amu.o src/bsp.o src/unicorn_ethdrv.o
MODEM_LIB = modem_lib/modem_ant_USB_LINUX.o.regparm3

unicorn_usb_eth-objs := $(OBJS) $(MODEM_LIB)

.SUFFIXES: .cpp .o

.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@


all:
	$(MAKE) -C $(KERNELDIR) M=$(MODDIR)
	
clean:
	$(MAKE) -C $(KERNELDIR) M=$(MODDIR) clean

install:
	mkdir -p /lib/modules/$(shell uname -r)/extra
	install -m 644 unicorn_usb_eth.ko /lib/modules/$(shell uname -r)/extra
	/sbin/depmod -ae

