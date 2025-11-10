#!/bin/bash

set -e # exit on error to prevent bad ipk from being generated

[ $# -eq 2 ] || {
    echo "Usage: make_package.sh VEHICLETYPE VEHICLE_BINARY"
    exit 1
}

VEHICLETYPE="$1"
VEHICLE_BINARY="$2"

[ -d $VEHICLETYPE ] || {
    echo "Vehicle directory $VEHICLETYPE not found"
    exit 1
}

# get version numbers
FW_MAJOR=$(grep FW_MAJOR $VEHICLETYPE/version.h | cut -d' ' -f3)
FW_MINOR=$(grep FW_MINOR $VEHICLETYPE/version.h | cut -d' ' -f3)
FW_PATCH=$(grep FW_PATCH $VEHICLETYPE/version.h | cut -d' ' -f3)
GIT_VERSION=$(git rev-parse HEAD | cut -c1-8)

VERSION="${FW_MAJOR}.${FW_MINOR}.${FW_PATCH}-${GIT_VERSION}"

cd libraries/AP_HAL_QURT/packaging

cat pkg/control/control.in | sed "s/FW_VERSION/$VERSION/g" > pkg/control/control

echo "Package Name: " $PACKAGE
echo "version Number: " $VERSION

################################################################################
# variables
################################################################################
PACKAGE=$(cat pkg/control/control | grep "Package" | cut -d' ' -f 2)

DATA_DIR=pkg/data
CONTROL_DIR=pkg/control
DEB_DIR=pkg/DEB

################################################################################
# start with a little cleanup to remove old files
################################################################################
# remove data directory where 'make install' installed to
rm -rf $DATA_DIR
mkdir $DATA_DIR

# remove deb packaging folders
rm -rf $DEB_DIR

################################################################################
## install compiled stuff into data directory
################################################################################

if [ -f ../../../build/QURT/ardupilot ] && \
   [ -f ../../../build/QURT/bin/$VEHICLE_BINARY ]; then

	# Copy the SLPI DSP AP library
    mkdir -p $DATA_DIR/usr/lib/rfsa/adsp
	cp ../../../build/QURT/bin/$VEHICLE_BINARY $DATA_DIR/usr/lib/rfsa/adsp/ArduPilot.so

    # Install executables
	mkdir -p $DATA_DIR/usr/bin
	cp ../../../build/QURT/ardupilot $DATA_DIR/usr/bin
	cp ../ap_host/service/voxl-ardupilot $DATA_DIR/usr/bin
	chmod a+x $DATA_DIR/usr/bin/ardupilot
	chmod a+x $DATA_DIR/usr/bin/voxl-ardupilot

    # Create necessary directories for ArduPilot operation
	mkdir -p $DATA_DIR/data/APM

	# Install default parameter files
	cp ../../../Tools/Frame_params/ModalAI/*.parm $DATA_DIR/data/APM

	mkdir -p $DATA_DIR/etc/systemd/system/
	cp ../ap_host/service/voxl-ardupilot.service $DATA_DIR/etc/systemd/system/

else
	echo "Error: Build artifacts not found"
	exit 1
fi

################################################################################
# make a DEB package
################################################################################

echo "starting building Debian Package"

## make a folder dedicated to Deb building and copy the requires debian-binary file in
mkdir $DEB_DIR

## copy the control stuff in
cp -rf $CONTROL_DIR/ $DEB_DIR/DEBIAN
cp -rf $DATA_DIR/*   $DEB_DIR

DEB_NAME="${PACKAGE}_${VEHICLETYPE}_${VERSION}_arm64.deb"
dpkg-deb --root-owner-group --build "${DEB_DIR}" "${DEB_NAME}"

echo "DONE"
