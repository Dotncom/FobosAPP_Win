cmake_minimum_required(VERSION 3.10)
project(FobosAPP VERSION 3.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif()

if(WIN32 AND NOT DEFINED Qt5_DIR AND EXISTS "C:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5")
    set(Qt5_DIR "C:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5" CACHE PATH "Qt5 CMake package directory")
endif()

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Network)
if(NOT WIN32)
    find_package(Qt5 COMPONENTS Multimedia QUIET)
    find_package(OpenGL REQUIRED)
    find_package(Threads REQUIRED)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(FOBOS_STANDARD_ROOT "${CMAKE_SOURCE_DIR}/fobos" CACHE PATH "Path to standard libfobos headers/library")
set(FOBOS_AGILE_ROOT "${CMAKE_SOURCE_DIR}/fobos_agile" CACHE PATH "Path to agile libfobos_sdr headers/library")
set(FFTW_ROOT "${CMAKE_SOURCE_DIR}/fftw-3.3.5-dll64" CACHE PATH "Path to FFTW headers/library")

find_path(FOBOS_STANDARD_INCLUDE_DIR
    NAMES fobos.h
    HINTS "${FOBOS_STANDARD_ROOT}" "${FOBOS_STANDARD_ROOT}/include"
)
find_path(FOBOS_AGILE_INCLUDE_DIR
    NAMES fobos_sdr.h
    HINTS "${FOBOS_AGILE_ROOT}" "${FOBOS_AGILE_ROOT}/include"
)
find_path(FFTW_INCLUDE_DIR
    NAMES fftw3.h
    HINTS "${FFTW_ROOT}" "${FFTW_ROOT}/include"
)

find_library(FOBOS_STANDARD_LIBRARY
    NAMES fobos libfobos
    HINTS "${FOBOS_STANDARD_ROOT}" "${FOBOS_STANDARD_ROOT}/lib" "${FOBOS_STANDARD_ROOT}/x64" "${FOBOS_STANDARD_ROOT}/x86"
)
find_library(FOBOS_AGILE_LIBRARY
    NAMES fobos_sdr libfobos_sdr
    HINTS "${FOBOS_AGILE_ROOT}" "${FOBOS_AGILE_ROOT}/lib"
)
find_library(FFTW_FLOAT_LIBRARY
    NAMES fftw3f libfftw3f-3
    HINTS "${FFTW_ROOT}" "${FFTW_ROOT}/lib"
)

foreach(required_path
        FOBOS_STANDARD_INCLUDE_DIR
        FOBOS_AGILE_INCLUDE_DIR
        FFTW_INCLUDE_DIR
        FOBOS_STANDARD_LIBRARY
        FOBOS_AGILE_LIBRARY
        FFTW_FLOAT_LIBRARY)
    if(NOT DEFINED ${required_path} OR "${${required_path}}" STREQUAL "")
        message(FATAL_ERROR "Missing ${required_path}. Set FOBOS_STANDARD_ROOT, FOBOS_AGILE_ROOT, or FFTW_ROOT, or install the dependency system-wide.")
    endif()
endforeach()

set(SOURCES
    src/main.cpp
    src/main.h
    src/dataprocessor.cpp
    src/dataprocessor.h
    src/diagnosticlogging.h
    src/dmrdecoder.cpp
    src/dmrdecoder.h
    src/digitaldecoder.cpp
    src/digitaldecoder.h
    src/frequencycontrol.cpp
    src/frequencycontrol.h
    src/iqbuffer.cpp
    src/iqbuffer.h
    src/networkcontroller.cpp
    src/networkcontroller.h
    src/playbackmanager.cpp
    src/playbackmanager.h
    src/remoteaudioplayer.cpp
    src/remoteaudioplayer.h
    src/recordingmanager.cpp
    src/recordingmanager.h
    src/radiosettings.h
    src/videoprocessor.cpp
    src/videoprocessor.h
    src/videowidget.cpp
    src/videowidget.h
    src/fft.cpp
    src/fft.h
    src/audioprocessor.cpp
    src/audioprocessor.h
    src/MyGraphWidget.cpp
    src/MyGraphWidget.h
    src/MyWaterfallWidget.cpp
    src/MyWaterfallWidget.h
    src/scalewidget.cpp
    src/scalewidget.h
)

add_executable(FobosAPP ${SOURCES})

target_include_directories(FobosAPP PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${FOBOS_STANDARD_INCLUDE_DIR}
    ${FOBOS_AGILE_INCLUDE_DIR}
    ${FFTW_INCLUDE_DIR}
)

target_link_libraries(FobosAPP PRIVATE
    ${FOBOS_STANDARD_LIBRARY}
    ${FOBOS_AGILE_LIBRARY}
    ${FFTW_FLOAT_LIBRARY}
    Qt5::Core
    Qt5::Widgets
    Qt5::Network
)

if(WIN32)
    target_link_libraries(FobosAPP PRIVATE winmm opengl32)
else()
    target_link_libraries(FobosAPP PRIVATE OpenGL::GL Threads::Threads)
endif()

if(NOT WIN32 AND Qt5Multimedia_FOUND)
    target_link_libraries(FobosAPP PRIVATE Qt5::Multimedia)
    target_compile_definitions(FobosAPP PRIVATE FOBOSAPP_HAS_QT_AUDIO=1)
elseif(NOT WIN32)
    message(WARNING "Qt5 Multimedia was not found. Linux audio playback will be disabled until the Qt Multimedia development package is installed.")
endif()

install(TARGETS FobosAPP RUNTIME DESTINATION bin)

if(UNIX AND NOT APPLE)
    install(FILES packaging/linux/99-fobos-sdr.rules
            DESTINATION share/FobosAPP/udev)
    install(FILES packaging/linux/fobosapp.desktop.in
            RENAME FobosAPP.desktop
            DESTINATION share/applications)
endif()
