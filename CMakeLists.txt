cmake_minimum_required(VERSION 3.10)

# to skip cmake to test a simple executable on a host
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

project(at32f425_usb_bootloader C CXX ASM)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)

## version information
set(VERSION_MAJOR 0 CACHE STRING "Project major version number.")
set(VERSION_MINOR 1 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")


if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
	if (NOT CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "" FORCE)
	endif()
endif()


message(STATUS "Building project: ${PROJECT_NAME}")

message(STATUS "Start src cmake")

include(${CMAKE_SOURCE_DIR}/cmake/toolchain.cmake)

set(MCU_ARCH cortex-m4)
set(MCU_NAME AT32F425R8T7)

add_compile_options(-mfloat-abi=soft)
add_compile_options(-mcpu=${MCU_ARCH} -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_compile_options(-DEMBEDDED -D${MCU_NAME} -DUSE_STDPERIPH_DRIVER -DAT_TAOBAO_F425)

file(GLOB APP_SOURCES "${CMAKE_SOURCE_DIR}/app/src/*.c")

file(GLOB COMMON_SOURCES_AT32F425
	"${CMAKE_SOURCE_DIR}/lib/src/cmsis/gcc/startup_at32f425.s"
	"${CMAKE_SOURCE_DIR}/lib/src/cmsis/system_at32f425.c"
	"${CMAKE_SOURCE_DIR}/lib/src/at32f425_acc.c"
	"${CMAKE_SOURCE_DIR}/lib/src/at32f425_crc.c"
	"${CMAKE_SOURCE_DIR}/lib/src/at32f425_crm.c"
	"${CMAKE_SOURCE_DIR}/lib/src/at32f425_exint.c"
	"${CMAKE_SOURCE_DIR}/lib/src/at32f425_flash.c"
	"${CMAKE_SOURCE_DIR}/lib/src/at32f425_gpio.c"
	"${CMAKE_SOURCE_DIR}/lib/src/at32f425_misc.c"
	"${CMAKE_SOURCE_DIR}/lib/src/at32f425_usb.c"
)

file(GLOB USB_DRIVER_SOURCES
	"${CMAKE_SOURCE_DIR}/usb/driver/src/usb_core.c"
	"${CMAKE_SOURCE_DIR}/usb/driver/src/usbd_core.c"
	"${CMAKE_SOURCE_DIR}/usb/driver/src/usbd_int.c"
	"${CMAKE_SOURCE_DIR}/usb/driver/src/usbd_sdr.c"
)

file(GLOB USBD_IAP_CLASS_SOURCES
	"${CMAKE_SOURCE_DIR}/usb/class/src/hid_iap_class.c"
	"${CMAKE_SOURCE_DIR}/usb/class/src/hid_iap_desc.c"
)

include_directories(
	"${CMAKE_SOURCE_DIR}/lib/include/cmsis"
	"${CMAKE_SOURCE_DIR}/lib/include"
	"${CMAKE_SOURCE_DIR}/usb/driver/include"
	"${CMAKE_SOURCE_DIR}/usb/class/include"
	"${CMAKE_CURRENT_SOURCE_DIR}/app/include"
)

# print summary of configurations
message(STATUS "MCU_ARCH: ${MCU_ARCH}")
message(STATUS "MCU_NAME: ${MCU_NAME}")
message(STATUS "Specified C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Linker script: ${LINKER_SCRIPT}")

list(APPEND SOURCES
	${APP_SOURCES}
	${COMMON_SOURCES_AT32F425}
	${USB_DRIVER_SOURCES}
	${USBD_IAP_CLASS_SOURCES}
)

message(STATUS "SOURCES:  ${SOURCES}")

set(LINKER_SCRIPT
	"${CMAKE_SOURCE_DIR}/lib/src/cmsis/gcc/linker/AT32F425x8_FLASH.ld"
)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-specs=nosys.specs)

message(STATUS "CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

target_link_options(${PROJECT_NAME}.elf PRIVATE -T ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_CXX_COMPILER_ID}_${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_CXX_COMPILER_ID}_${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
	COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
	COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}"
)
