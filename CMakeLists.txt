# I target a recent cmake, it shouldn't be a problem on a dev machine
cmake_minimum_required(VERSION 3.11)

set(F_CPU 8000000UL)
set(BAUD 9600)

# Platform details
set(MCU atmega328p)

# Debug/upload from avrdude
set(PROG_TYPE avrispmkII)

# AVR Fuses, must be in concordance with your hardware and F_CPU
# http://eleccelerator.com/fusecalc/fusecalc.php?chip=atmega328p
set(E_FUSE 0xfd)
set(H_FUSE 0xda)
set(L_FUSE 0xfd)
set(LOCK_BIT 0xff)

set(CMAKE_CROSSCOMPILING true)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)
SET(TARGET avr)

# AVR toolchain
set(TOOLCHAIN_PATH C:/Users/Kate/.platformio/packages/toolchain-atmelavr)
set(AVRDUDE_PATH C:/Users/Kate/Developer/avrdude)

SET(CMAKE_C_COMPILER_TARGET ${TARGET})
SET(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/avr-gcc.exe)
SET(CMAKE_CXX_COMPILER_TARGET ${TARGET})
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/avr-g++.exe)
SET(CMAKE_ASM_COMPILER_TARGET ${TARGET})
SET(CMAKE_ASM_COMPILER ${TOOLCHAIN_PATH}/bin/avr-gcc.exe)

set(CXX_STANDARD 20)
set(C_STANDARD 17)

enable_language(C ASM)

# Project name
project("Burst")

# Product filename
set(PRODUCT_NAME burst)

# Pass defines to compiler
add_definitions(
    -DF_CPU=${F_CPU}
    -DBAUD=${BAUD}
)
# mmcu MUST be passed to both the compiler and linker, this handles the linker
set(CMAKE_EXE_LINKER_FLAGS -mmcu=${MCU})

set(AVR_PATH ${TOOLCHAIN_PATH}/avr)
include_directories(SYSTEM ${AVR_PATH}/include)
link_directories(${AVR_PATH}/lib)

# Arduino
set(ARDUINO_PATH C:/Users/Kate/.platformio/packages/framework-arduino-avr)
set(ARDUINO_CORE ${ARDUINO_PATH}/cores/arduino)
set(ARDUINO_LIBRARIES_PATH ${ARDUINO_PATH}/libraries)

# Board
set(ARDUINO_VARIANT standard)

# Built-in libraries to include
set(ARDUINO_USE_BUILTIN_LIBRARIES EEPROM)

file(GLOB ARDUINO_CORE_SRC_C ${ARDUINO_CORE}/*.cpp)
file(GLOB ARDUINO_CORE_SRC_CXX ${ARDUINO_CORE}/*.c)
file(GLOB ARDUINO_CORE_SRC_ASM ${ARDUINO_CORE}/*.S)
set(ARDUINO_SRC ${ARDUINO_CORE_SRC_C} ${ARDUINO_CORE_SRC_CXX} ${ARDUINO_CORE_SRC_ASM})

# Clang-only
#set(CMAKE_CXX_FLAGS "--gcc-toolchain=${TOOLCHAIN_PATH} --sysroot=${AVR_PATH}")
#set(CMAKE_C_FLAGS "--gcc-toolchain=${TOOLCHAIN_PATH} --sysroot=${AVR_PATH}")

set(CMAKE_CXX_FLAGS "-std=gnu++20 -fno-rtti -Wno-volatile -fpermissive -fno-exceptions -fno-threadsafe-statics")
add_compile_options(
    -nostdlib
    -mmcu=${MCU} # MCU
    -Os # optimize
    -Wall # enable warnings
    -Wno-main
    -Wundef
    -pedantic
    #-Werror
    -Wfatal-errors
    -Wl,--relax,--gc-sections
    -Wno-array-bounds

    # Arduino specific
    #-Wno-unknown-attributes
    #-Wno-gnu-zero-variadic-macro-arguments
    #-Wno-strict-prototypes
    #-Wno-gnu-anonymous-struct
    #-Wno-implicit-exception-spec-mismatch

    # Debug and flags
    -g
    -gdwarf-2
    -funsigned-char # a few optimizations
    -fpack-struct
    -fshort-enums
    -ffunction-sections
    -fdata-sections
)

# Create the executable target
add_executable(${PRODUCT_NAME} src/BURST.cpp)
target_include_directories(${PRODUCT_NAME} PRIVATE include)
target_link_libraries(${PRODUCT_NAME} m c)

# User libs here
file(GLOB AVR_LIBSTDCPP_SRC lib/avr-libstdcpp/src/*.cc)
add_library(avr-libstdcpp STATIC ${AVR_LIBSTDCPP_SRC})
target_include_directories(avr-libstdcpp PUBLIC lib/avr-libstdcpp/include)
target_link_libraries(${PRODUCT_NAME} avr-libstdcpp)

# Arduino core library
add_library(Arduino STATIC ${ARDUINO_SRC})
target_include_directories(Arduino PUBLIC ${ARDUINO_PATH}/variants/${ARDUINO_VARIANT})
target_include_directories(Arduino PUBLIC ${ARDUINO_CORE})
target_link_libraries(${PRODUCT_NAME} Arduino)

# Arduino built-in libraries
foreach (ARDUINO_LIBRARY IN LISTS ARDUINO_USE_BUILTIN_LIBRARIES)
    #source
    cmake_path(SET ARDUINO_LIBRARY_SRC_PATH ${ARDUINO_LIBRARIES_PATH}/${ARDUINO_LIBRARY}/src/)

    file(GLOB ARDUINO_LIBRARY_SRC_C ${ARDUINO_LIBRARY_SRC_PATH}/*.c)
    file(GLOB ARDUINO_LIBRARY_SRC_C ${ARDUINO_LIBRARY_SRC_PATH}/**/*.c)
    file(GLOB ARDUINO_LIBRARY_SRC_CXX ${ARDUINO_LIBRARY_SRC_PATH}/*.cpp)
    file(GLOB ARDUINO_LIBRARY_SRC_CXX ${ARDUINO_LIBRARY_SRC_PATH}/**/*.cpp)
    set(ARDUINO_LIBRARY_SRC ${ARDUINO_LIBRARY_SRC_C} ${ARDUINO_LIBRARY_SRC_CXX})

    #header
    cmake_path(SET ARDUINO_LIBRARY_INCLUDE_PATH ${ARDUINO_LIBRARIES_PATH}/${ARDUINO_LIBRARY}/src/ )

    # Create and link library
    if(ARDUINO_LIBRARY_SRC STREQUAL "") 
        add_library(${ARDUINO_LIBRARY} STATIC ${ARDUINO_LIBRARY_SRC})
        target_include_directories(${ARDUINO_LIBRARY} PUBLIC ${ARDUINO_LIBRARY_INCLUDE_PATH})
    else()
        add_library(${ARDUINO_LIBRARY} INTERFACE)
        target_include_directories(${ARDUINO_LIBRARY} INTERFACE ${ARDUINO_LIBRARY_INCLUDE_PATH})
    endif()    

    target_link_libraries(${PRODUCT_NAME} ${ARDUINO_LIBRARY})
endforeach()

# Rename the output to .elf as we will create multiple files
set_target_properties(${PRODUCT_NAME} PROPERTIES OUTPUT_NAME ${PRODUCT_NAME}.elf)

# Strip binary for upload
add_custom_target(strip ALL ${TOOLCHAIN_PATH}/bin/avr-strip.exe ${PRODUCT_NAME}.elf DEPENDS ${PRODUCT_NAME})

# Transform binary into hex file, we ignore the eeprom segments in the step
add_custom_target(hex ALL ${TOOLCHAIN_PATH}/bin/avr-objcopy.exe -R .eeprom -O ihex ${PRODUCT_NAME}.elf ${PRODUCT_NAME}.hex DEPENDS strip)
# Transform binary into hex file, this is the eeprom part (empty if you don't
# use eeprom static variables)
add_custom_target(eeprom ${TOOLCHAIN_PATH}/bin/avr-objcopy.exe -j .eeprom  --set-section-flags=.eeprom="alloc,load"  --change-section-lma .eeprom=0 -O ihex ${PRODUCT_NAME}.elf ${PRODUCT_NAME}.eep DEPENDS strip)

# Upload the firmware with avrdude
add_custom_target(upload ${AVRDUDE_PATH}/avrdude.exe  -c ${PROG_TYPE} -p ${MCU} -U flash:w:${PRODUCT_NAME}.hex DEPENDS hex)

# Upload the eeprom with avrdude
add_custom_target(upload_eeprom ${AVRDUDE_PATH}/avrdude.exe -c ${PROG_TYPE} -p ${MCU}  -U eeprom:w:${PRODUCT_NAME}.eep DEPENDS eeprom)

# Burn fuses
add_custom_target(fuses ${AVRDUDE_PATH}/avrdude.exe -c ${PROG_TYPE} -p ${MCU}  -U lfuse:w:${L_FUSE}:m -U hfuse:w:${H_FUSE}:m -U efuse:w:${E_FUSE}:m -U lock:w:${LOCK_BIT}:m )

# Clean extra files
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${PRODUCT_NAME}.hex;${PRODUCT_NAME}.eeprom;${PRODUCT_NAME}.lst")
