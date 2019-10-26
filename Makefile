.DEFAULT_GOAL 	 := build
PROJECT_NAME     := mpl115a2

.PHONY: clean build

build:
	cargo build

clean:
	cargo clean
