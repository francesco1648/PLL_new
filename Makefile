# Basic settings
SKETCH_PATH = $(CURDIR)
SKETCH_NAME = $(notdir $(SKETCH_PATH))

# Board configuration
BOARD_FQBN ?= rp2040:rp2040:rpipico
OUTPUT_DIR = $(CURDIR)/build/output
BUILD_DIR = $(CURDIR)/build
LIBS_DIR = $(CURDIR)/lib
INCLUDE_DIR = $(CURDIR)/include

LIBRARY_PATHS = $(wildcard $(LIBS_DIR)/*/src)
LIBRARY_FLAGS = $(addprefix --library ,$(LIBRARY_PATHS))

INCLUDE_PATHS = $(INCLUDE_DIR) $(LIBRARY_PATHS)
CFLAGS += $(foreach dir, $(INCLUDE_PATHS), -I$(dir))
CXXFLAGS += $(foreach dir, $(INCLUDE_PATHS), -I$(dir))

SUCCESS_SYMBOL = " Compilation completed successfully! "
ERROR_SYMBOL = " Compilation error! "
COMPILATION_SYMBOL = " Compilation in progress... "

MODULE_DEFINE ?= "MK1_MOD1"
DESTINATION ?=  'E:\'

MODULE =
define print_green
	@pwsh  -Command "Write-Host '$1' -ForegroundColor Green"
endef

define print_red
	@pwsh  -Command "Write-Host '$1' -ForegroundColor Red"
endef

PORT ?= $(shell arduino-cli board list | findstr "Raspberry Pi Pico" | for /f "tokens=1" %%a in ('more') do @echo %%a)

.DEFAULT:
	@echo "Invalid command: '$@'"
	@echo "Use 'make help' to see the list of available commands."
	@$(MAKE) help

# Compilation
compile: clean_all
	$(call print_green, $(COMPILATION_SYMBOL))
	@arduino-cli compile --fqbn $(BOARD_FQBN) --build-path $(BUILD_DIR) $(SKETCH_PATH) --output-dir $(OUTPUT_DIR) $(LIBRARY_FLAGS) \
		$(foreach dir, $(INCLUDE_PATHS), --build-property "compiler.cpp.extra_flags=-I$(dir) -D$(MODULE_DEFINE)")

compile_fast:
	@arduino-cli compile --fqbn $(BOARD_FQBN) "$(SKETCH_PATH)"

# Upload .bin file
upload:
	@echo "Check that you have entered the correct COM port. The current COM port is: $(PORT)"
	@if exist "$(OUTPUT_DIR)/$(SKETCH_NAME).ino.bin" ( \
		echo "Uploading .bin file to Raspberry Pi Pico..." & \
		arduino-cli upload -p $(PORT) --fqbn $(BOARD_FQBN) --input-dir $(OUTPUT_DIR) $(SKETCH_NAME).ino.bin \
	) else ( \
		echo " .bin file not found. Run 'make compile' before uploading the code." \
	)

# Upload .uf2 file in BOOTSEL mode
upload_bootsel:
	@if exist "$(OUTPUT_DIR)/$(SKETCH_NAME).ino.uf2" ( \
		echo "Uploading .uf2 file to Raspberry Pi Pico..." && \
		powershell -Command "Copy-Item '$(OUTPUT_DIR)/$(SKETCH_NAME).ino.uf2' -Destination  $(DESTINATION)  -Force" || ( \
		    echo "Error while uploading, please make sure the pico is in BOOTSEL mode and is recognized as $(DESTINATION) storage drive. If it is recognized as another drive, please change the 'Destination' field of the upload_bootsel command"; \
		) \
	) else ( \
		echo ".uf2 file not found. Run 'make compile' before uploading the code." \
	)


# Clean the build folder
clean_all:
	@echo BUILD_DIR is: "$(BUILD_DIR)"
	@if exist "$(BUILD_DIR)\output" ( \
		echo The build folder exists. & \
		rd /s /q "$(BUILD_DIR)" & \
		echo Build folder content removed. \
	) else ( \
		echo The output folder does not exist. \
	)


clean_output:
	@echo "Cleaning in progress..."
	@if exist "$(BUILD_DIR)/output" ( \
		echo "Removing files in the build folder..." \
		rd /s /q "$(BUILD_DIR)/output" \
		echo "Content of the output folder removed." \
	) else ( \
		echo "The output folder does not exist." \
	)
	$(call print_green, "Content of the output folder cleaned.")




# Serial monitor
monitor:
	arduino-cli monitor -p $(PORT) -c baudrate=115200

# Command guide
help:
	@echo "Available commands:"
	@echo "  make compile       - Compile the project. E possibile aggiungere il parametro MODULE_DEFINE per definire un modulo specifico"
	@echo "  make compile_fast  - Fast compilation without additional libraries"
	@echo "  make upload        - Upload the project to Raspberry Pi Pico"
	@echo "  make upload_bootsel - Upload the .uf2 file manually to E:/"
	@echo "  make monitor       - Start the serial monitor"
	@echo "  make all           - Compile and upload the project in one step"
	@echo "  make clean         - Clean compilation files"
	@echo "  make help          - Show this guide"
	@echo "  make auto_com_port - Automatically detect the list of COM port of the Raspberry Pi Pico"
	@echo "  make port          - List all available COM ports"

# Print detected COM port
auto_com_port:
	@echo "The automatically detected COM port is: $(PORT)"

# List all available COM ports
port:
	@echo "List of COM ports detected by the system:"
	@arduino-cli board list
