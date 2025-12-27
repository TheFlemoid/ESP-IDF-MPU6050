cmake_minimum_required (VERSION 3.5)

file(GLOB_RECURSE SOURCE_FILES src/*.c)
file(GLOB_RECURSE HEADER_FILES src/*.h)

if (NOT DEFINED COMPONENT_DIR)

    project(MPU6050)

    include_directories(src)

else()

    idf_component_register(SRCS ${SOURCE_FILES}
                           INCLUDE_DIRS
                               "src"
                           REQUIRES
                               "driver freertos")

endif()
