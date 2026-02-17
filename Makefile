# Minimal Makefile for RTL8188EUS USB Detection Driver

# Module name
obj-m += rtl8188eu_minimal.o

# Object files that make up the module
rtl8188eu_minimal-objs := rtl8188eu_minimal_main.o rtl8188eu_phy.o rtl8188eu_phy_tables.o

# Kernel build directory
KDIR := /lib/modules/$(shell uname -r)/build

# Current directory
PWD := $(shell pwd)

# Default target - build the module
all:
	@echo "Building minimal RTL8188EUS driver..."
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	@echo "Build complete! Module: rtl8188eu_minimal.ko"

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	@echo "Clean complete!"

# Install module (copy to kernel modules directory)
install:
	@echo "Installing module..."
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a
	@echo "Module installed!"

# Show module info
info:
	@echo "Module information:"
	@modinfo rtl8188eu_minimal.ko

# Load the module
load:
	@echo "Loading module..."
	sudo insmod rtl8188eu_minimal.ko
	@echo "Module loaded! Check dmesg for output."

# Unload the module
unload:
	@echo "Unloading module..."
	sudo rmmod rtl8188eu_minimal
	@echo "Module unloaded!"

# Show kernel messages
dmesg:
	@dmesg | tail -20

.PHONY: all clean install info load unload dmesg
