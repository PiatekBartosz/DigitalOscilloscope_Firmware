PROJECT_NAME = main
BUILD_DIR = _build
DEPLOY_DIR = _deploy

BOARD = "nucleo_h723zg"

#TODO: add later
# OVERLAY = "src/boards/stm32h7.overlay"
#  -DDTC_OVERLAY_FILE=$(OVERLAY) 

.PHONY: all cmake build clean run rebuild

# Default target: build the project
all: build deploy

# Build the project
build:
	mkdir -p $(BUILD_DIR)
	cmake -B _build -S . -GNinja -DBOARD=${BOARD} && ninja -C _build

# Clean the build directory
clean:
	rm -rf $(BUILD_DIR)

# Run the compiled executable
deploy: build
	mkdir -p $(DEPLOY_DIR)
	cp $(BUILD_DIR)/zephyr/zephyr.elf $(DEPLOY_DIR)/

# Rebuild the project from scratch
rebuild: clean all