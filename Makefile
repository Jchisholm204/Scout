.PHONY: all build clean

all: build

clean:
	rm -r ./build ./install ./log
	rm ./compile_commands.json

build:
	./scripts/build.sh
