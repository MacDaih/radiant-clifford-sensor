UID := $(shell id -u)
GID := $(shell id -g)

all: build-clifford build-services

build-clifford:
	gcc -o thermo ./src/clifford.c -lwiringPi -lsdk

run-clifford:
	modprobe i2c-dev
	chmod +x ./thermo
	./thermo & 

