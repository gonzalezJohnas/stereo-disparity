cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(KEYWORD stereo_disparity)
set(YARP_INCLUDE_CV /home/jonas/yarp/src/libYARP_cv/include)
project($KEYWORD)


find_package(OpenCV REQUIRED)
find_package(YARP  REQUIRED COMPONENTS sig cv )

FIND_PACKAGE(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")


include_directories(${KEYWORD}
        ${OpenCV_INCLUDE_DIRS}
        ${YARP_INCLUDE_CV}
        )

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD}
            ${folder_source}
            ${folder_header}
            )

    TARGET_LINK_LIBRARIES(${KEYWORD}
            ${OpenCV_LIBS}
            ${YARP_LIBRARIES}
            )

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)


add_subdirectory(app)
icubcontrib_add_uninstall_target()
