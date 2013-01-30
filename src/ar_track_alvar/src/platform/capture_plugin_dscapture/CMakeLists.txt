# include opencv library
include_directories(${OpenCV_INCLUDE_DIRS})

# include alvar library
include_directories(${ALVAR_INCLUDE_DIRS})

# include DSCapture library
include_directories(${DSCAPTURE_INCLUDE_DIRS})

# ensure plugin api is exported when building library
add_definitions(-DALVAR_Capture_Plugin_DSCapture_BUILD)

# define plugin library source files to compile
set(ALVARCapturePluginDSCapture_HEADERS
    CapturePluginDSCapture.h
)
set(ALVARCapturePluginDSCapture_SOURCES
    CapturePluginDSCapture.cpp
)

# add source files to plugin library and set name and properties
add_library(ALVARCapturePluginDSCapture MODULE
    ${ALVARCapturePluginDSCapture_HEADERS}
    ${ALVARCapturePluginDSCapture_SOURCES}
)
set_target_properties(ALVARCapturePluginDSCapture PROPERTIES OUTPUT_NAME alvarcaptureplugindscapture${ALVAR_VERSION_NODOTS})
set_target_properties(ALVARCapturePluginDSCapture PROPERTIES DEBUG_POSTFIX d)

# link plugin library target to required libraries
target_link_libraries(ALVARCapturePluginDSCapture
    ALVAR
    ALVARPlatform
    ${OpenCV_LIBRARIES}
    optimized ${DSCAPTURE_LIBRARIES}
    debug ${DSCAPTURE_LIBRARIES_DEBUG}
)

# install headers
if(ALVAR_PACKAGE MATCHES sdk)
    install(FILES ${ALVARCapturePluginDSCapture_HEADERS} DESTINATION include/platform/capture_plugin_dscapture)
endif(ALVAR_PACKAGE MATCHES sdk)

# install target
install(TARGETS ALVARCapturePluginDSCapture DESTINATION bin/alvarplugins)

# hack to setup environment runtime path
if(NOT ${MSVC_IDE})
    list(APPEND ALVAR_RUNTIME
        ${CMAKE_CURRENT_BINARY_DIR}\;
    )
else(NOT ${MSVC_IDE})
    list(APPEND ALVAR_RUNTIME
        ${CMAKE_CURRENT_BINARY_DIR}/Debug\;
        ${CMAKE_CURRENT_BINARY_DIR}/Release\;
    )
endif(NOT ${MSVC_IDE})
set(ALVAR_RUNTIME ${ALVAR_RUNTIME} PARENT_SCOPE)
