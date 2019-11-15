.DEFAULT_GOAL 	 := all
PROJECT_NAME     := mpl115a2

.PHONY: all clean build test

all: build test

build:
	@cargo build

clean:
	@cargo clean

test:
	@RUST_BACKTRACE=1 cargo test
