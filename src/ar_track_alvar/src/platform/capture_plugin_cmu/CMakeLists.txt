# include opencv library
include_directories(${OpenCV_INCLUDE_DIRS})

# include alvar library
include_directories(${ALVAR_INCLUDE_DIRS})

# include CMU library
include_directories(${CMU_INCLUDE_DIRS})

# ensure plugin api is exported when building library
add_definitions(-DALVAR_Capture_Plugin_CMU_BUILD)

# define plugin library source files to compile
set(ALVARCapturePluginCMU_HEADERS
    CapturePluginCmu.h
)
set(ALVARCapturePluginCMU_SOURCES
    CapturePluginCmu.cpp
)

# add source files to plugin library and set name and properties
add_library(ALVARCapturePluginCMU MODULE
    ${ALVARCapturePluginCMU_HEADERS}
    ${ALVARCapturePluginCMU_SOURCES}
)
set_target_properties(ALVARCapturePluginCMU PROPERTIES OUTPUT_NAME alvarcaptureplugincmu${ALVAR_VERSION_NODOTS})
set_target_properties(ALVARCapturePluginCMU PROPERTIES DEBUG_POSTFIX d)

# link plugin library target to required libraries
target_link_libraries(ALVARCapturePluginCMU
    ALVAR
    ALVARPlatform
    ${OpenCV_LIBRARIES}
    ${CMU_LIBRARIES}
)

# install headers
if(ALVAR_PACKAGE MATCHES sdk)
    install(FILES ${ALVARCapturePluginCMU_HEADERS} DESTINATION include/platform/capture_plugin_cmu)
endif(ALVAR_PACKAGE MATCHES sdk)

# install target
install(TARGETS ALVARCapturePluginCMU DESTINATION bin/alvarplugins)

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
