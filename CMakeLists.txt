cmake_minimum_required(VERSION 3.10)
project(FobosAPP VERSION 4.5.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif()

if(WIN32 AND NOT DEFINED Qt5_DIR AND EXISTS "C:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5")
    set(Qt5_DIR "C:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5" CACHE PATH "Qt5 CMake package directory")
endif()

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Network SerialPort)
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

option(FOBOS_ENABLE_MBELIB_NEO "Enable mbelib-neo DMR AMBE vocoder support" ON)
set(FOBOS_HAS_MBELIB_NEO OFF)
if(FOBOS_ENABLE_MBELIB_NEO AND EXISTS "${CMAKE_SOURCE_DIR}/third_party/mbelib-neo/CMakeLists.txt")
    set(MBELIB_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(MBELIB_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(MBELIB_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(MBELIB_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
    set(MBELIB_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)
    add_subdirectory(third_party/mbelib-neo EXCLUDE_FROM_ALL)
    set(FOBOS_HAS_MBELIB_NEO ON)
else()
    message(STATUS "mbelib-neo not found or disabled; DMR voice synthesis will be unavailable.")
endif()

set(SOURCES
    src/main.cpp
    src/main.h
    src/appdiagnostics.cpp
    src/appdiagnostics.h
    src/apphelp.cpp
    src/apphelp.h
    src/dataprocessor.cpp
    src/dataprocessor.h
    src/diagnosticlogging.h
    src/dmrdecoder.cpp
    src/dmrdecoder.h
    src/dmrvocoder.cpp
    src/dmrvocoder.h
    src/dsdneobridge.cpp
    src/dsdneobridge.h
    src/gophertrunkbridge.cpp
    src/gophertrunkbridge.h
    src/dmrhunterdetector.cpp
    src/dmrhunterdetector.h
    src/digitalvideohunterdetector.cpp
    src/digitalvideohunterdetector.h
    src/digitaldecoder.cpp
    src/digitaldecoder.h
    src/bladerfbackend.cpp
    src/bladerfbackend.h
    src/frequencycontrol.cpp
    src/frequencycontrol.h
    src/finetunewidget.cpp
    src/finetunewidget.h
    src/fobosbackend.cpp
    src/fobosbackend.h
    src/fpvhunterdetector.cpp
    src/fpvhunterdetector.h
    src/gnssacquisition.cpp
    src/gnssacquisition.h
    src/gnssqthhelpers.cpp
    src/gnssqthhelpers.h
    src/gnsssignalmonitor.cpp
    src/gnsssignalmonitor.h
    src/iqbuffer.cpp
    src/iqbuffer.h
    src/networkcontroller.cpp
    src/networkcontroller.h
    src/playbackmanager.cpp
    src/playbackmanager.h
    src/presethelpers.cpp
    src/presethelpers.h
    src/presetmanager.cpp
    src/qthlocator.cpp
    src/qthlocator.h
    src/qthmapwidget.cpp
    src/qthmapwidget.h
    src/remoteaudioplayer.cpp
    src/remoteaudioplayer.h
    src/recordingmanager.cpp
    src/recordingmanager.h
    src/radiosettings.h
    src/receiverbackend.h
    src/receiverbackendregistry.cpp
    src/receiverbackendregistry.h
    src/rtlsdrbackend.cpp
    src/rtlsdrbackend.h
    src/spectrumhuntercontrols.cpp
    src/spectrumhuntercontrols.h
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
    src/scanvisualassembler.cpp
    src/scanvisualassembler.h
    src/soapysdrbackend.cpp
    src/soapysdrbackend.h
    src/app_resources.qrc
)

if(WIN32)
    list(APPEND SOURCES packaging/windows/fobosapp.rc)
endif()

add_executable(FobosAPP ${SOURCES})

target_include_directories(FobosAPP PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/FobosDMRVoiceBackend-gpl/include
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
    Qt5::SerialPort
)

if(FOBOS_HAS_MBELIB_NEO)
    target_link_libraries(FobosAPP PRIVATE mbe_neo::mbe_static)
    target_compile_definitions(FobosAPP PRIVATE FOBOSAPP_HAS_MBELIB_NEO=1)
endif()

if(WIN32)
    target_link_libraries(FobosAPP PRIVATE winmm opengl32 winhttp)
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
    install(FILES packaging/icons/fobosapp.png
            RENAME fobosapp.png
            DESTINATION share/icons/hicolor/256x256/apps)
    install(FILES packaging/linux/fobosapp.desktop.in
            RENAME FobosAPP.desktop
            DESTINATION share/applications)
endif()
