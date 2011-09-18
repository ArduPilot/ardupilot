set(LIB_NAME AP_Common)
	
set(${LIB_NAME}_SRCS 
        AP_Common.cpp
        AP_Loop.cpp
        AP_MetaClass.cpp
        AP_Var.cpp
        AP_Var_menufuncs.cpp
        c++.cpp
        menu.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  
        AP_Common.h
        AP_Loop.h
        AP_MetaClass.h
        AP_Var.h
        AP_Test.h
        c++.h
        AP_Vector.h
)
	
include_directories(
 
  -
  ${CMAKE_SOURCE_DIR}/libraries/AP_Common
	${CMAKE_SOURCE_DIR}/libraries/FastSerial
#	
	)

set(${LIB_NAME}_BOARD mega2560)

generate_arduino_library(${LIB_NAME})
