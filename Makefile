# Raspberry Pi DAQ (CAN encoder/inclinometer + ADS1263 ADC -> SQLite)
#
#   sudo apt install libsqlite3-dev       # + bcm2835 library (http://www.airspayce.com/mikem/bcm2835/)
#   make                                  # build/daq, build/can_module_test, build/can_2ch_load_test
#   make ADC_KERNEL_CS=1                  # if GPIO22 is declared as a kernel cs-gpio (see README)
#   make test                             # host unit tests (no hardware needed)

CC       ?= gcc
CXX      ?= g++
BUILD    ?= build
ADC_KERNEL_CS ?= 0
ADC_SPI_DEVICE ?= /dev/spidev0.1

WARN     := -Wall -Wextra
override CPPFLAGS += -Iinclude -Idrivers -DADC_KERNEL_CS=$(ADC_KERNEL_CS) -DADC_SPI_DEVICE='"$(ADC_SPI_DEVICE)"'
CFLAGS   += -O2 -g $(WARN) -std=gnu11
CXXFLAGS += -O2 -g $(WARN) -std=c++17
LDLIBS_HW := -lbcm2835
LDLIBS_DB := -lsqlite3

DRIVER_OBJS := $(BUILD)/ADS1263.o $(BUILD)/DEV_Config.o
HEADERS := $(wildcard include/*.hpp) $(wildcard drivers/*.h)

all: $(BUILD)/daq $(BUILD)/can_module_test $(BUILD)/can_2ch_load_test

$(BUILD):
	mkdir -p $@

$(BUILD)/%.o: drivers/%.c $(HEADERS) | $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: src/%.cpp $(HEADERS) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD)/daq: $(BUILD)/daq_main.o $(BUILD)/db_writer.o $(DRIVER_OBJS)
	$(CXX) $^ -o $@ -pthread $(LDLIBS_HW) $(LDLIBS_DB)

$(BUILD)/can_module_test: tools/can_module_test.cpp $(HEADERS) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ -pthread

$(BUILD)/can_2ch_load_test: tools/can_2ch_load_test.cpp $(HEADERS) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ -pthread

# ---- host tests (x86 ok) ----
$(BUILD)/test_sensor_parser: tests/test_sensor_parser.cpp include/sensor_parser.hpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(BUILD)/test_db_writer: tests/test_db_writer.cpp src/db_writer.cpp include/db_writer.hpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_db_writer.cpp src/db_writer.cpp -o $@ -pthread $(LDLIBS_DB)

test: $(BUILD)/test_sensor_parser $(BUILD)/test_db_writer
	$(BUILD)/test_sensor_parser
	$(BUILD)/test_db_writer

clean:
	rm -rf $(BUILD)

.PHONY: all test clean
