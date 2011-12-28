#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_SENSORS_MT9T013)	+= mt9t013.o

obj-$(CONFIG_DS2482)		+= ds2482.o
ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

#//Div2D5-OwenHuang-BSP4040_Sensors_Porting-00+{
obj-$(CONFIG_SENSORS_LTR502ALS)	+= ltr502als.o
#//Div2D5-OwenHuang-BSP4040_Sensors_Porting-00+}
