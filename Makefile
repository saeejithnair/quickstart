# To install the dependencies and build the config and messages, use:
# CONFIG=bot_quickstart CONFIG_MSGS=bot_quickstart_msgs make

# To only build the config and messages, use:
# CONFIG=bot_quickstart CONFIG_MSGS=bot_quickstart_msgs make build

.PHONY: all debug setup clean build

CONFIG ?= bot_quickstart
CONFIG_MSGS ?= bot_quickstart_msgs
CFG_PATH = $(shell pwd)/config/$(CONFIG).toml

all: install build

build: 
	python3 utils/parse_toml_config.py config/$(CONFIG).toml
	python3 utils/generate_mqtt_messages.py config/$(CONFIG_MSGS).toml

clean:
	# nothing to clean

install:
	pip3 install -e .
