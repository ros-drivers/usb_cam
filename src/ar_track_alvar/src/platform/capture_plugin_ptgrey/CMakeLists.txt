# include opencv library
include_directories(${OpenCV_INCLUDE_DIRS})

# include alvar library
include_directories(${ALVAR_INCLUDE_DIRS})

# include PTGrey FlyCapture2 library
include_directories(${FLYCAPTURE2_INCLUDE_DIRS})

# ensure plugin api is exported when building library
add_definitions(-DALVAR_Capture_Plugin_Ptgrey_BUILD)

# define plugin library source files to compile
set(ALVARCapturePluginPtgrey_HEADERS
    CapturePluginPtgrey.h
)
set(ALVARCapturePluginPtgrey_SOURCES
    CapturePluginPtgrey.cpp
)

# add source files to plugin library and set name and properties
add_library(ALVARCapturePluginPtgrey MODULE
    ${ALVARCapturePluginPtgrey_HEADERS}
    ${ALVARCapturePluginPtgrey_SOURCES}
)
set_target_properties(ALVARCapturePluginPtgrey PROPERTIES OUTPUT_NAME alvarcapturepluginptgrey${ALVAR_VERSION_NODOTS})
set_target_properties(ALVARCapturePluginPtgrey PROPERTIES DEBUG_POSTFIX d)

# link plugin library target to required libraries
target_link_libraries(ALVARCapturePluginPtgrey
    ALVAR
    ALVARPlatform
    ${OpenCV_LIBRARIES}
    optimized ${FLYCAPTURE2_LIBRARIES}
    debug ${FLYCAPTURE2_LIBRARIES_DEBUG}
)

# install headers
if(ALVAR_PACKAGE MATCHES sdk)
    install(FILES ${ALVARCapturePluginPtgrey_HEADERS} DESTINATION include/platform/capture_plugin_ptgrey)
endif(ALVAR_PACKAGE MATCHES sdk)

# install target
install(TARGETS ALVARCapturePluginPtgrey DESTINATION bin/alvarplugins)

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
