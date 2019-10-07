obj-m += kraken.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install: all
	sudo cp kraken.ko /lib/modules/$(shell uname -r)/kernel/drivers/hwmon

update: all
	sudo modprobe -r kraken; \
		make install && \
		sudo depmod && \
		sudo modprobe kraken && \
		dmesg | tail

.PHONY: all clean install update
