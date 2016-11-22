cmake_minimum_required(VERSION 3.3)
project(cvdrone)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wc++11-narrowing -Wdeprecated-declarations")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


find_package(PkgConfig)


MACRO(FFMPEG_FIND varname shortname headername)

    IF(NOT WIN32)
        PKG_CHECK_MODULES(PC_${varname} ${shortname})

        FIND_PATH(${varname}_INCLUDE_DIR "${shortname}/${headername}"
                HINTS ${PC_${varname}_INCLUDEDIR} ${PC_${varname}_INCLUDE_DIRS}
                NO_DEFAULT_PATH
                )
    ELSE()
        FIND_PATH(${varname}_INCLUDE_DIR "${shortname}/${headername}")
    ENDIF()

    IF(${varname}_INCLUDE_DIR STREQUAL "${varname}_INCLUDE_DIR-NOTFOUND")
        message(STATUS "look for newer strcture")
        IF(NOT WIN32)
            PKG_CHECK_MODULES(PC_${varname} "lib${shortname}")

            FIND_PATH(${varname}_INCLUDE_DIR "lib${shortname}/${headername}"
                    HINTS ${PC_${varname}_INCLUDEDIR} ${PC_${varname}_INCLUDE_DIRS}
                    NO_DEFAULT_PATH
                    )
        ELSE()
            FIND_PATH(${varname}_INCLUDE_DIR "lib${shortname}/${headername}")
            IF(${${varname}_INCLUDE_DIR} STREQUAL "${varname}_INCLUDE_DIR-NOTFOUND")
                #Desperate times call for desperate measures
                MESSAGE(STATUS "globbing...")
                FILE(GLOB_RECURSE ${varname}_INCLUDE_DIR "/ffmpeg*/${headername}")
                MESSAGE(STATUS "found: ${${varname}_INCLUDE_DIR}")
                IF(${varname}_INCLUDE_DIR)
                    GET_FILENAME_COMPONENT(${varname}_INCLUDE_DIR "${${varname}_INCLUDE_DIR}" PATH)
                    GET_FILENAME_COMPONENT(${varname}_INCLUDE_DIR "${${varname}_INCLUDE_DIR}" PATH)
                ELSE()
                    SET(${varname}_INCLUDE_DIR "${varname}_INCLUDE_DIR-NOTFOUND")
                ENDIF()
            ENDIF()
        ENDIF()
    ENDIF()


    IF(${${varname}_INCLUDE_DIR} STREQUAL "${varname}_INCLUDE_DIR-NOTFOUND")
        MESSAGE(STATUS "Can't find includes for ${shortname}...")
    ELSE()
        MESSAGE(STATUS "Found ${shortname} include dirs: ${${varname}_INCLUDE_DIR}")

        #		GET_DIRECTORY_PROPERTY(FFMPEG_PARENT DIRECTORY ${${varname}_INCLUDE_DIR} PARENT_DIRECTORY)
        GET_FILENAME_COMPONENT(FFMPEG_PARENT ${${varname}_INCLUDE_DIR} PATH)
        MESSAGE(STATUS "Using FFMpeg dir parent as hint: ${FFMPEG_PARENT}")

        IF(NOT WIN32)
            FIND_LIBRARY(${varname}_LIBRARIES NAMES ${shortname}
                    HINTS ${PC_${varname}_LIBDIR} ${PC_${varname}_LIBRARY_DIR} ${FFMPEG_PARENT})
        ELSE()
            #			FIND_PATH(${varname}_LIBRARIES "${shortname}.dll.a" HINTS ${FFMPEG_PARENT})
            FILE(GLOB_RECURSE ${varname}_LIBRARIES "${FFMPEG_PARENT}/*${shortname}.lib")
            # GLOBing is very bad... but windows sux, this is the only thing that works
        ENDIF()

        IF(${varname}_LIBRARIES STREQUAL "${varname}_LIBRARIES-NOTFOUND")
            MESSAGE(STATUS "look for newer structure for library")
            FIND_LIBRARY(${varname}_LIBRARIES NAMES lib${shortname}
                    HINTS ${PC_${varname}_LIBDIR} ${PC_${varname}_LIBRARY_DIR} ${FFMPEG_PARENT})
        ENDIF()


        IF(${varname}_LIBRARIES STREQUAL "${varname}_LIBRARIES-NOTFOUND")
            MESSAGE(STATUS "Can't find lib for ${shortname}...")
        ELSE()
            MESSAGE(STATUS "Found ${shortname} libs: ${${varname}_LIBRARIES}")
        ENDIF()


        IF(NOT ${varname}_INCLUDE_DIR STREQUAL "${varname}_INCLUDE_DIR-NOTFOUND"
                AND NOT ${varname}_LIBRARIES STREQUAL ${varname}_LIBRARIES-NOTFOUND)

            MESSAGE(STATUS "found ${shortname}: include ${${varname}_INCLUDE_DIR} lib ${${varname}_LIBRARIES}")
            SET(FFMPEG_${varname}_FOUND 1)
            SET(FFMPEG_${varname}_INCLUDE_DIRS ${${varname}_INCLUDE_DIR})
            SET(FFMPEG_${varname}_LIBS ${${varname}_LIBRARIES})
        ELSE()
            MESSAGE(STATUS "Can't find ${shortname}")
        ENDIF()

    ENDIF()

ENDMACRO(FFMPEG_FIND)

FFMPEG_FIND(LIBAVFORMAT avformat avformat.h)
FFMPEG_FIND(LIBAVDEVICE avdevice avdevice.h)
FFMPEG_FIND(LIBAVCODEC  avcodec  avcodec.h)
FFMPEG_FIND(LIBAVUTIL   avutil   avutil.h)
FFMPEG_FIND(LIBSWSCALE  swscale  swscale.h)

SET(FFMPEG_FOUND "NO")
IF   (FFMPEG_LIBAVFORMAT_FOUND AND
        FFMPEG_LIBAVDEVICE_FOUND AND
        FFMPEG_LIBAVCODEC_FOUND AND
        FFMPEG_LIBAVUTIL_FOUND AND
        FFMPEG_LIBSWSCALE_FOUND
        )


    SET(FFMPEG_FOUND "YES")

    SET(FFMPEG_INCLUDE_DIRS ${FFMPEG_LIBAVFORMAT_INCLUDE_DIRS})

    SET(FFMPEG_LIBRARY_DIRS ${FFMPEG_LIBAVFORMAT_LIBRARY_DIRS})

    SET(FFMPEG_LIBRARIES
            ${FFMPEG_LIBAVFORMAT_LIBS}
            ${FFMPEG_LIBAVDEVICE_LIBS}
            ${FFMPEG_LIBAVCODEC_LIBS}
            ${FFMPEG_LIBAVUTIL_LIBS}
            ${FFMPEG_LIBSWSCALE_LIBS}
            )

ELSE ()

    MESSAGE(STATUS "Could not find FFMPEG")

ENDIF()


#find_package(FFmpeg REQUIRED)
if (FFMPEG_FOUND)
    message("Found FFMPEG/LibAV libraries")
    include_directories(${FFMPEG_INCLUDE_DIR})
    set(libraries ${FFMPEG_LIBRARIES})
#    target_link_libraries (cvdrone ${FFMPEG_LIBRARIES})
else (FFMPEG_FOUND)
    message("Can't find libavcodec, libavformat or libavutil. Add them!")
endif (FFMPEG_FOUND)

#target_link_libraries(cvdrone "-lm")
#target_link_libraries(cvdrone "-lavresample")


set(SRC_FILES src/main.cpp src/control.cpp src/structures.h )
#add_subdirectory(src/manual)

file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE SRC_FILES ${SRC_FILES} ${PROJECT_SOURCE_DIR}/src/*.h)
#file(GLOB_RECURSE SRC_FILES ${SRC_FILES} ${PROJECT_SOURCE_DIR}/src/lineFollowing/*.h)

file(GLOB_RECURSE SRC_FILES ${SRC_FILES} ${PROJECT_SOURCE_DIR}/src/*.lib)

add_executable(cvdrone ${SRC_FILES})

set(libraries ${libraries} ${OpenCV_LIBS})
target_link_libraries(cvdrone ${libraries})