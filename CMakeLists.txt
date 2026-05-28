cmake_minimum_required(VERSION 3.10)
project(FobosAPP VERSION 2.9.0)

set(CMAKE_CXX_STANDARD 17)
set(Qt5_DIR "C:/Qt/5.15.2/msvc2019_64/")
set(Qt5_DIR "C:/Qt/5.15.2/msvc2019_64/bin") # Укажите свой путь к Qt
set(Qt5_DIR "C:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5")
set(Qt5_DIR "C:/Qt/5.15.2/msvc2019_64/lib")

include_directories(${CMAKE_SOURCE_DIR}/fobos/x64)
include_directories(${CMAKE_SOURCE_DIR}/fobos)
include_directories(${CMAKE_SOURCE_DIR}/fobos_agile)
include_directories(${CMAKE_SOURCE_DIR}/fftw-3.3.5-dll64)
include_directories(${CMAKE_SOURCE_DIR}/src)
#include_directories(${CMAKE_SOURCE_DIR}/libusb-1.0.27/libusb-1.0.27/libusb/)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Network)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Подключение заголовков и исходников
set(SOURCES
    #libusb-1.0.27/libusb-1.0.27/libusb/libusb.h
    fobos/fobos.h
    fftw-3.3.5-dll64/fftw3.h
    src/main.cpp
    src/main.h
    src/dataprocessor.cpp
    src/dataprocessor.h
    src/diagnosticlogging.h
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

# Создание исполняемого файла
add_executable(FobosAPP ${SOURCES})

# Подключение библиотек Qt
target_link_directories(FobosAPP PRIVATE
    ${CMAKE_SOURCE_DIR}/fobos
    ${CMAKE_SOURCE_DIR}/fobos_agile
    ${CMAKE_BINARY_DIR}
)
target_link_libraries(FobosAPP PRIVATE fobos fobos_sdr libfftw3f-3 Qt5::Core Qt5::Widgets Qt5::Network winmm opengl32.lib)
