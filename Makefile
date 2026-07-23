PROJECT_NAME = main
debug ?= n
BUILD_ROOT = _build
DEPLOY_ROOT = _deploy

BOARD = nucleo_h723zg_dev/stm32h723xx
EXT_MODULE = $(PWD)/ext

JOBS := $(shell nproc)

VALID_DEBUG_VALUES := n y
ifeq ($(filter $(debug),$(VALID_DEBUG_VALUES)),)
$(error debug must be y or n)
endif

CONFIG = $(if $(filter y,$(debug)),debug,release)
BUILD_DIR = $(BUILD_ROOT)/$(CONFIG)
DEPLOY_DIR = $(DEPLOY_ROOT)/$(CONFIG)
CMAKE_BUILD_TYPE = $(if $(filter y,$(debug)),Debug,Release)
CONF_FILE = prj.conf;prj_$(CONFIG).conf

CLANG_FORMAT  ?= clang-format
FORMAT_DIRS    = src
FORMAT_PATTERN = \( -name "*.c" -o -name "*.h" \)

.DEFAULT_GOAL := all

.PHONY: all build deploy clean clean-config rebuild format format-check

all: build deploy

build:
	mkdir -p $(BUILD_DIR)
	export ZEPHYR_EXTRA_MODULES=$(EXT_MODULE) && \
	cmake -B $(BUILD_DIR) -S . -GNinja -DBOARD=$(BOARD) \
		-DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) \
		-DCONF_FILE="$(CONF_FILE)" && \
	ninja -C $(BUILD_DIR) -j$(JOBS)
	echo "$(CONFIG) build done!"

clean:
	rm -rf $(BUILD_ROOT) $(DEPLOY_ROOT)
	echo "All build and deploy artifacts removed!"

clean-config:
	rm -rf $(BUILD_DIR) $(DEPLOY_DIR)
	echo "$(CONFIG) build and deploy artifacts removed!"

deploy: build
	mkdir -p $(DEPLOY_DIR)
	cp $(BUILD_DIR)/zephyr/zephyr.elf $(DEPLOY_DIR)/
	echo "$(CONFIG) deploy done: $(DEPLOY_DIR)/zephyr.elf"

rebuild: clean-config build deploy
	echo "$(CONFIG) rebuild done!"

format:
	find $(FORMAT_DIRS) $(FORMAT_PATTERN) -print0 | xargs -0 $(CLANG_FORMAT) -i
	echo "Format done!"

format-check:
	find $(FORMAT_DIRS) $(FORMAT_PATTERN) -print0 | xargs -0 $(CLANG_FORMAT) --dry-run --Werror
	echo "Format check passed!"
