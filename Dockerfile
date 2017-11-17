FROM ubuntu:16.04
ADD . /ardupilot
WORKDIR /ardupilot

RUN apt-get update && apt-get upgrade -y && apt-get dist-upgrade -y && apt-get install lsb-release sudo software-properties-common python-software-properties -y
ENV USER=root
RUN bash -c "Tools/scripts/install-prereqs-ubuntu.sh -y && apt-get install gcc-arm-none-eabi -y"
