NAME=microfarmseesaw
BUILD_PATH=build
OPENOCD_CONFIG_FILE=openocdconfig/openocd.cfg

CHIP_FAMILY = SAMD09
CHIP_VARIANT = SAMD09D14A
LINKER_SCRIPT=scripts/samd09d14a_flash.ld

EXECUTABLE=$(BUILD_PATH)/$(NAME).bin

-include Makefile.user
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++

ifeq ($(DEBUG), 1)
ENABLE_LOGGING = -DENABLE_LOGGING
else
ENABLE_LOGGING =
endif

COMMON_FLAGS = -mthumb -mcpu=cortex-m0plus -Os -g3 -D$(CHIP_FAMILY) -D__$(CHIP_VARIANT)__ $(ENABLE_LOGGING)

WFLAGS = \
-Wall -Werror

SFLAGS = $(COMMON_FLAGS) \
-x assembler-with-cpp -c \
$(WFLAGS)
CFLAGS = $(COMMON_FLAGS) \
-x c -std=gnu99 -c \
-MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" \
$(WFLAGS)
CXXFLAGS = $(COMMON_FLAGS) \
-ffunction-sections -fdata-sections \
-fno-rtti -fno-exceptions -c -std=c++11 \
-MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" \
$(WFLAGS)

LDFLAGS= $(COMMON_FLAGS) \
-Wall -Werror -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common \
-Wl,--warn-section-align -Wl,--warn-unresolved-symbols \
-save-temps \
--specs=nano.specs --specs=nosys.specs

INCLUDES = -I. -I./include
INCLUDES += -I./boards/$(BOARD) -Ilib/cmsis/CMSIS/Include
INCLUDES += -I$(BUILD_PATH)

INCLUDES += -Ilib/samd09/include/
CSOURCES = Device_Startup/startup_samd09.c \
	Device_Startup/system_samd09.c

SOURCES = $(COMMON_SRC) \
	source/Microfarm.cpp \
	source/MicrofarmCommunications.cpp \
	source/main.cpp

FULL_SOURCES = $(SOURCES)

SOBJECTS = $(patsubst %.S,$(BUILD_PATH)/%.o,$(SSOURCES))
COBJECTS = $(patsubst %.c,$(BUILD_PATH)/%.o,$(CSOURCES))
OBJECTS = $(patsubst %.cpp,$(BUILD_PATH)/%.o,$(FULL_SOURCES))

all: dirs $(EXECUTABLE)

dirs:
	-@mkdir -p $(BUILD_PATH)
	-@mkdir -p $(BUILD_PATH)/source
	-@mkdir -p $(BUILD_PATH)/Device_Startup
	@cp $(OPENOCD_CONFIG_FILE) $(BUILD_PATH)/

$(EXECUTABLE): $(SOBJECTS) $(COBJECTS) $(OBJECTS)
	$(CC) -L$(BUILD_PATH) $(LDFLAGS) \
		 -T$(LINKER_SCRIPT) \
		 -Wl,-Map,$(BUILD_PATH)/$(NAME).map -o $(BUILD_PATH)/$(NAME).elf $(SOBJECTS) $(COBJECTS) $(OBJECTS)
	arm-none-eabi-objcopy -O binary $(BUILD_PATH)/$(NAME).elf $@
	@echo
	-@arm-none-eabi-size $(BUILD_PATH)/$(NAME).elf
	@echo

$(BUILD_PATH)/%.o: %.S $(wildcard include/*.h)
	echo "$<"
	$(CC) $(SFLAGS) $(BLD_EXTA_FLAGS) $(INCLUDES) $< -o $@

$(BUILD_PATH)/%.o: %.c $(wildcard include/*.h)
	echo "$<"
	$(CC) $(CFLAGS) $(BLD_EXTA_FLAGS) $(INCLUDES) $< -o $@

$(BUILD_PATH)/%.o: %.cpp $(wildcard include/*.h)
	echo "$<"
	$(CXX) $(CXXFLAGS) $(BLD_EXTA_FLAGS) $(INCLUDES) $< -o $@

clean:
	rm -rf build
