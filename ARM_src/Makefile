#
TARGET = my_soc_run

#
ALT_DEVICE_FAMILY ?= soc_cv_av
HWLIBS_ROOT = /home/ubuntu/Desktop/hwlib
CROSS_COMPILE = arm-linux-gnueabihf-
CFLAGS = -g -Wall -std=gnu++11  -D$(ALT_DEVICE_FAMILY) -I$(HWLIBS_ROOT)/$(ALT_DEVICE_FAMILY)   -I$(HWLIBS_ROOT)/ `pkg-config --cflags opencv`
LDFLAGS =  -g -Wall 
CC = $(CROSS_COMPILE)g++
ARCH= arm

build: $(TARGET)
$(TARGET): main.cpp plr.cpp
	$(CC) $(CFLAGS)   $^ -o $@  `pkg-config --libs opencv`

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *~ 
