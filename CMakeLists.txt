cmake_minimum_required(VERSION 3.10)
project(FobosAPP VERSION 4.5.1)

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
    src/mainwindow/main_appsettings.cpp
    src/mainwindow/main_controls.cpp
    src/mainwindow/main_devices.cpp
    src/mainwindow/main_digital.cpp
    src/mainwindow/main_dmr.cpp
    src/mainwindow/main_dmr_privacy.cpp
    src/mainwindow/main_gnss.cpp
    src/mainwindow/main_gnss_acquisition.cpp
    src/mainwindow/main_gnss_acquisition_tests.cpp
    src/mainwindow/main_gnss_map.cpp
    src/mainwindow/main_gnss_satellites.cpp
    src/mainwindow/main_gnss_serial.cpp
    src/mainwindow/main_hardware_controls.cpp
    src/mainwindow/main_localization.cpp
    src/mainwindow/main_localization_translations.cpp
    src/mainwindow/main_measurements.cpp
    src/mainwindow/main_network.cpp
    src/mainwindow/main_network_audio.cpp
    src/mainwindow/main_network_dialog.cpp
    src/mainwindow/main_network_iq.cpp
    src/mainwindow/main_network_spectrum.cpp
    src/mainwindow/main_presets.cpp
    src/mainwindow/main_recording.cpp
    src/mainwindow/main_scan.cpp
    src/mainwindow/main_scan_hunters.cpp
    src/mainwindow/main_settings.cpp
    src/mainwindow/main_settings_json.cpp
    src/mainwindow/main_settings_ui.cpp
    src/mainwindow/main_spectrum.cpp
    src/utils/appruntimeutils.cpp
    src/utils/appruntimeutils.h
    src/core/appdiagnostics.cpp
    src/core/appdiagnostics.h
    src/utils/appconstants.h
    src/utils/appsettingsutils.cpp
    src/utils/appsettingsutils.h
    src/ui/apphelp.cpp
    src/ui/apphelp.h
    src/core/dataprocessor.cpp
    src/core/dataprocessor.h
    src/core/diagnosticlogging.h
    src/utils/dmrbackendpaths.cpp
    src/utils/dmrbackendpaths.h
    src/digital/dmrdecoder.cpp
    src/digital/dmrdecoder.h
    src/digital/dmrvocoder.cpp
    src/digital/dmrvocoder.h
    src/utils/dmrprivacyutils.cpp
    src/utils/dmrprivacyutils.h
    src/digital/dsdneobridge.cpp
    src/digital/dsdneobridge.h
    src/digital/gophertrunkbridge.cpp
    src/digital/gophertrunkbridge.h
    src/digital/dmrhunterdetector.cpp
    src/digital/dmrhunterdetector.h
    src/digital/digitalvideohunterdetector.cpp
    src/digital/digitalvideohunterdetector.h
    src/digital/digitaldecoder.cpp
    src/digital/digitaldecoder.h
    src/backends/bladerfbackend.cpp
    src/backends/bladerfbackend.h
    src/ui/frequencycontrol.cpp
    src/ui/frequencycontrol.h
    src/ui/finetunewidget.cpp
    src/ui/finetunewidget.h
    src/backends/fobosbackend.cpp
    src/backends/fobosbackend.h
    src/digital/fpvhunterdetector.cpp
    src/digital/fpvhunterdetector.h
    src/gnss/gnssacquisition.cpp
    src/gnss/gnssacquisition.h
    src/gnss/gnssqthhelpers.cpp
    src/gnss/gnssqthhelpers.h
    src/utils/gnssserialutils.cpp
    src/utils/gnssserialutils.h
    src/gnss/gnsssignalmonitor.cpp
    src/gnss/gnsssignalmonitor.h
    src/core/iqbuffer.cpp
    src/core/iqbuffer.h
    src/network/networkcontroller.cpp
    src/network/networkcontroller.h
    src/audio/playbackmanager.cpp
    src/audio/playbackmanager.h
    src/presets/presethelpers.cpp
    src/presets/presethelpers.h
    src/presets/presetmanager.cpp
    src/gnss/qthlocator.cpp
    src/gnss/qthlocator.h
    src/gnss/qthmapwidget.cpp
    src/gnss/qthmapwidget.h
    src/audio/remoteaudioplayer.cpp
    src/audio/remoteaudioplayer.h
    src/audio/recordingmanager.cpp
    src/audio/recordingmanager.h
    src/core/radiosettings.h
    src/backends/receiverbackend.h
    src/backends/receiverbackendregistry.cpp
    src/backends/receiverbackendregistry.h
    src/utils/receiverdeviceutils.cpp
    src/utils/receiverdeviceutils.h
    src/backends/rtlsdrbackend.cpp
    src/backends/rtlsdrbackend.h
    src/utils/samplefileutils.cpp
    src/utils/samplefileutils.h
    src/ui/spectrumhuntercontrols.cpp
    src/ui/spectrumhuntercontrols.h
    src/digital/videoprocessor.cpp
    src/digital/videoprocessor.h
    src/digital/videowidget.cpp
    src/digital/videowidget.h
    src/core/fft.cpp
    src/core/fft.h
    src/utils/spectrumfftworker.cpp
    src/utils/spectrumfftworker.h
    src/audio/audioprocessor.cpp
    src/audio/audioprocessor.h
    src/ui/MyGraphWidget.cpp
    src/ui/MyGraphWidget.h
    src/ui/MyWaterfallWidget.cpp
    src/ui/MyWaterfallWidget.h
    src/utils/modulationutils.cpp
    src/utils/modulationutils.h
    src/ui/scalewidget.cpp
    src/ui/scalewidget.h
    src/scan/scanvisualassembler.cpp
    src/scan/scanvisualassembler.h
    src/utils/scanvisualutils.cpp
    src/utils/scanvisualutils.h
    src/backends/soapysdrbackend.cpp
    src/backends/soapysdrbackend.h
    src/utils/tuningutils.cpp
    src/utils/tuningutils.h
    src/app_resources.qrc
)

if(WIN32)
    list(APPEND SOURCES packaging/windows/fobosapp.rc)
endif()

add_executable(FobosAPP ${SOURCES})

target_include_directories(FobosAPP PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/audio
    ${CMAKE_SOURCE_DIR}/src/backends
    ${CMAKE_SOURCE_DIR}/src/core
    ${CMAKE_SOURCE_DIR}/src/digital
    ${CMAKE_SOURCE_DIR}/src/gnss
    ${CMAKE_SOURCE_DIR}/src/mainwindow
    ${CMAKE_SOURCE_DIR}/src/network
    ${CMAKE_SOURCE_DIR}/src/presets
    ${CMAKE_SOURCE_DIR}/src/scan
    ${CMAKE_SOURCE_DIR}/src/ui
    ${CMAKE_SOURCE_DIR}/src/utils
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
