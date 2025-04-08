
cmake_minimum_required(VERSION 2.8.6)
project(lanefreeplugin)
set(CMAKE_BUILD_TYPE Debug)
if(WIN32)
	set(CMAKE_SUPPRESS_REGENERATION true)
endif()
#Bring the headers into the project
include_directories(src  ${PROJECT_BINARY_DIR})

#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.cpp")



#output library export file *.lib and
#output macro definitions include file
if(WIN32)
	include (GenerateExportHeader)
	#Generate the shared library from the sources
	add_library(libLaneFreePlugin SHARED ${SOURCES})
else()
	#Generate the shared library from the sources (linux automatically assigns a lib at the beginning of the library)
	add_library(LaneFreePlugin SHARED ${SOURCES})
endif()


if(WIN32)
	

	GENERATE_EXPORT_HEADER (libLaneFreePlugin
	    BASE_NAME libLaneFreePlugin
	    EXPORT_MACRO_NAME libLaneFreePlugin_EXPORT    
	    EXPORT_FILE_NAME libLaneFreePlugin_Export.h
	    STATIC_DEFINE libLaneFreePlugin_BUILT_AS_STATIC
	)
endif()
