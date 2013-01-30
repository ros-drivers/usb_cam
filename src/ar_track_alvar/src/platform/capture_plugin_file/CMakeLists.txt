# include opencv library
include_directories(${OpenCV_INCLUDE_DIRS})

# include alvar library
include_directories(${ALVAR_INCLUDE_DIRS})

# ensure plugin api is exported when building library
add_definitions(-DALVAR_Capture_Plugin_File_BUILD)

# define plugin library source files to compile
set(ALVARCapturePluginFile_HEADERS
    CapturePluginFile.h
)
set(ALVARCapturePluginFile_SOURCES
    CapturePluginFile.cpp
)

# add source files to plugin library and set name and properties
add_library(ALVARCapturePluginFile MODULE
    ${ALVARCapturePluginFile_HEADERS}
    ${ALVARCapturePluginFile_SOURCES}
)
set_target_properties(ALVARCapturePluginFile PROPERTIES OUTPUT_NAME alvarcapturepluginfile${ALVAR_VERSION_NODOTS})
set_target_properties(ALVARCapturePluginFile PROPERTIES DEBUG_POSTFIX d)

# link plugin library target to required libraries
target_link_libraries(ALVARCapturePluginFile
    ALVAR
    ALVARPlatform
    ${OpenCV_LIBRARIES}
)

# install headers
if(ALVAR_PACKAGE MATCHES sdk)
    install(FILES ${ALVARCapturePluginFile_HEADERS} DESTINATION include/platform/capture_plugin_file)
endif(ALVAR_PACKAGE MATCHES sdk)

# install target
install(TARGETS ALVARCapturePluginFile DESTINATION bin/alvarplugins)

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
