#=============================================================================#
# Author: Sebastian Rohde                                                      #
# Date:   30.08.2011                                                          #
#=============================================================================#


#====================================================================#
#  Settings                                                          #
#====================================================================#
set(FIRMWARE_NAME apo)

set(${FIRMWARE_NAME}_BOARD ${BOARD})               # Arduino Target board

set(${FIRMWARE_NAME}_SKETCHES
	apo.pde	
	)  # Firmware sketches
	
set(${FIRMWARE_NAME}_SRCS 
	)  # Firmware sources
		
set(${FIRMWARE_NAME}_HDRS 
        ControllerPlane.h
        ControllerQuad.h
        PlaneEasystar.h
        QuadArducopter.h
        QuadMikrokopter.h
	)  # Firmware sources

set(${FIRMWARE_NAME}_LIBS
    m
    APO
    FastSerial
    AP_Common
    GCS_MAVLink
    AP_GPS
    APM_RC
    AP_DCM
    AP_ADC
    AP_Compass
    AP_IMU
    AP_RangeFinder
    APM_BMP085
	ModeFilter
)
	

#${CONSOLE_PORT}
set(${FIRMWARE_NAME}_PORT COM2)            # Serial upload port
set(${FIRMWARE_NAME}_SERIAL putty -serial COM2 -sercfg 57600,8,n,1,X )  # Serial terminal cmd
	
include_directories(
${CMAKE_SOURCE_DIR}/libraries/APO
${CMAKE_SOURCE_DIR}/libraries/AP_Common
${CMAKE_SOURCE_DIR}/libraries/FastSerial
${CMAKE_SOURCE_DIR}/libraries/ModeFilter
${CMAKE_SOURCE_DIR}/libraries/AP_Compass
${CMAKE_SOURCE_DIR}/libraries/AP_RangeFinder
${CMAKE_SOURCE_DIR}/libraries/AP_GPS
${CMAKE_SOURCE_DIR}/libraries/AP_IMU
${CMAKE_SOURCE_DIR}/libraries/AP_ADC
${CMAKE_SOURCE_DIR}/libraries/AP_DCM
${CMAKE_SOURCE_DIR}/libraries/APM_RC
${CMAKE_SOURCE_DIR}/libraries/GCS_MAVLink
${CMAKE_SOURCE_DIR}/libraries/APM_BMP085
)
#====================================================================#
#  Target generation                                                 #
#====================================================================#
generate_arduino_firmware(${FIRMWARE_NAME})

install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_NAME}.hex 
	DESTINATION bin
	)
