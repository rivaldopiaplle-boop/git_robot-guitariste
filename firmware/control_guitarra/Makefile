# Micrologiciel du robot guitariste (STM32F411RE, Nucleo), hors de STM32CubeIDE.
# Mêmes options que la configuration Debug de l'IDE, pour que la chaîne
# d'intégration compile exactement ce que compile le poste de travail.
#
#   make            -> build/control_guitarra.elf, .bin, et la taille mémoire
#   make clean

CIBLE   = control_guitarra
BUILD   = build
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

MCU     = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFINES = -DUSE_HAL_DRIVER -DSTM32F411xE
INCLUDES = -ICore/Inc \
           -IDrivers/STM32F4xx_HAL_Driver/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
           -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/CMSIS/Include \
           -IDrivers/CMSIS_DSP/Include -IDrivers/CMSIS_DSP/PrivateInclude
CFLAGS  = $(MCU) $(DEFINES) $(INCLUDES) -std=gnu11 -Os -g \
          -ffunction-sections -fdata-sections -Wall --specs=nano.specs
LDFLAGS = $(MCU) -TSTM32F411RETX_FLASH.ld --specs=nosys.specs --specs=nano.specs \
          -Wl,-Map=$(BUILD)/$(CIBLE).map -Wl,--gc-sections -static \
          -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group

# CMSIS-DSP : la détection de la note (FFT, moyenne, maximum). Comme dans
# l'IDE, chaque fonction est compilée séparément : les fichiers qui les
# regroupent ont leurs inclusions commentées, ils ne produisent rien.
FAMILLES_DSP = BasicMathFunctions CommonTables FilteringFunctions                StatisticsFunctions SupportFunctions TransformFunctions
DSP = $(foreach f,$(FAMILLES_DSP),$(wildcard Drivers/CMSIS_DSP/Source/$(f)/*.c))
SOURCES_C = $(wildcard Core/Src/*.c) $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c) $(DSP)
SOURCES_S = $(wildcard Core/Startup/*.s)
OBJETS    = $(addprefix $(BUILD)/,$(SOURCES_C:.c=.o) $(SOURCES_S:.s=.o))

all: $(BUILD)/$(CIBLE).bin
	@$(SIZE) $(BUILD)/$(CIBLE).elf

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(CIBLE).elf: $(OBJETS)
	$(CC) $(OBJETS) $(LDFLAGS) -o $@

$(BUILD)/$(CIBLE).bin: $(BUILD)/$(CIBLE).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
