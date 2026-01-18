PROJECT_NAME = main
BUILD_DIR = _build
DEPLOY_DIR = _deploy

BOARD = "nucleo_h723zg_dev"
EXT_MODULE = $(PWD)/ext

.PHONY: all cmake build clean run rebuild

# Default target: build the project
all: build deploy

# Build the project
build:
	mkdir -p $(BUILD_DIR)
	export ZEPHYR_EXTRA_MODULES=$(EXT_MODULE) && \
	cmake -B $(BUILD_DIR) -S . -GNinja -DBOARD=${BOARD} && \
	ninja -C $(BUILD_DIR)
	echo "Build done!"

# Clean the build directory
clean:
	rm -rf $(BUILD_DIR)
	echo "Clean done!"

# Run the compiled executable
deploy: build
	mkdir -p $(DEPLOY_DIR)
	cp $(BUILD_DIR)/zephyr/zephyr.elf $(DEPLOY_DIR)/
	echo "Deploy done!"

# Rebuild the project from scratch
rebuild: clean all
	echo "Rebuild done!"