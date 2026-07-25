all: build

build:
	mkdir -p com/MAVLink
	mavgen.py --lang Java --wire-protocol=1.0 ../../../../message_definitions/v1.0/ardupilotmega.xml -o com/MAVLink
	mkdir -p build
	javac -d build test.java com/MAVLink/*.java com/MAVLink/common/*.java com/MAVLink/ardupilotmega/*.java

clean:
	rm -rf build com/MAVLink
