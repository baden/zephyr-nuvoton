
# set(NUVOTON_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "NUVOTON root directory")

# add_subdirectory(nuvoton)

add_subdirectory(drivers)

if(CONFIG_LEGACY_INCLUDE_PATH)
  zephyr_include_directories(include/zephyr)
endif()

zephyr_include_directories(include)
# if(EXISTS ${SOC_DIR}/${ARCH}/CMakeLists.txt)
#   add_subdirectory(${SOC_DIR}/${ARCH} soc/${ARCH})
# # else()
# #   add_subdirectory(${SOC_DIR}/${ARCH}/${SOC_PATH} soc/${ARCH}/${SOC_PATH})
# endif()
