KERNEL_SRC = ../linux/build

EXTRA_CFLAGS := $(EC) -I$(KERNEL_SRC)/../include 
EXTRA_CFLAGS += -mcpu=cortex-a53 -march=armv8-a+crc -mtune=cortex-a53
EXTRA_CFLAGS += -O3
#EXTRA_CFLAGS += -fno-stack-protector
#EXTRA_CFLAGS += -fno-stack-clash-protection
#EXTRA_CFLAGS += -fno-semantic-interposition
#EXTRA_CFLAGS += -fno-math-errno
EXTRA_CFLAGS += -Wno-unused-result
EXTRA_CFLAGS += -Wno-declaration-after-statement

#EXTRA_CFLAGS += -DDEBUG_VSOUND
EXTRA_CFLAGS += -DCONFIG_SMPD_OPTION_RPI_DAC_32BIT_786KHZ

OBJS    = vsound.o
obj-m := $(OBJS)

all:
	make ARCH=arm64 -C $(KERNEL_SRC) M=${shell pwd} modules

clean:
	-rm -f *.[oas] *.ko *.mod.c .*.d .*.tmp .*.cmd *.symvers *.order *.mod
	-rm -rf .tmp_versions
