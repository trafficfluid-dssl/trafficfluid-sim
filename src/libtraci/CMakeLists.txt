if (${CMAKE_VERSION} VERSION_GREATER 3.13.0)
    # UseSWIG generates standard target names
    cmake_policy(SET CMP0078 NEW)
endif()
if (${CMAKE_VERSION} VERSION_GREATER 3.14.0)
    # pass module name to swig compiler
    cmake_policy(SET CMP0086 NEW)
endif()
set(libtracistatic_STAT_SRCS
    Connection.cpp
	Connection.h
	Domain.h
#    Edge.cpp
#    InductionLoop.cpp
#    Junction.cpp
#    Lane.cpp
#    LaneArea.cpp
#    MultiEntryExit.cpp
#    Person.cpp
#    POI.cpp
#    Polygon.cpp
    Route.cpp
#    Calibrator.cpp
#    BusStop.cpp
#    ParkingArea.cpp
#    ChargingStation.cpp
#    RouteProbe.cpp
#    Rerouter.cpp
#    VariableSpeedSign.cpp
#    MeanData.cpp
#    OverheadWire.cpp
    Simulation.cpp
#    TrafficLight.cpp
#    Vehicle.cpp
#    VehicleType.cpp
)

add_library(libtracistatic STATIC ${libtracistatic_STAT_SRCS})
add_dependencies(libtracistatic generate-version-h)
set_property(TARGET libtracistatic PROPERTY FOLDER "libtraci")

if (MSVC)
    set(libtcpip foreign_tcpip ws2_32)
else()
    set(libtcpip foreign_tcpip)
endif()

if(SWIG_FOUND)
    include(${SWIG_USE_FILE})
    set_source_files_properties(libtraci.i PROPERTIES CPLUSPLUS ON)
    file(GLOB libtraci_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/../libsumo/*.h")
    if(ENABLE_JAVA_BINDINGS)
        find_package(Java)
        find_package(JNI)
        if(JAVA_FOUND AND JNI_FOUND)
            include(UseJava)
            include_directories(${JNI_INCLUDE_DIRS})
            set(CMAKE_SWIG_FLAGS -package org.eclipse.sumo.libtraci)
            set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/java)
            set(SWIG_MODULE_libtracijni_EXTRA_DEPS ${libtraci_HEADERS})
            if (${CMAKE_VERSION} VERSION_GREATER 3.8.0)
                SWIG_ADD_LIBRARY(libtracijni LANGUAGE java SOURCES libtraci.i)
            else()
                SWIG_ADD_MODULE(libtracijni java libtraci.i)
            endif()
            if (MSVC OR APPLE)
                swig_link_libraries(libtracijni libtracistatic ${libtcpip})
            else()
                set_source_files_properties(${swig_generated_file_fullname} PROPERTIES COMPILE_FLAGS "-Wno-strict-aliasing")
                swig_link_libraries(libtracijni -Wl,--whole-archive libtracistatic ${libtcpip} -Wl,--no-whole-archive)
            endif()
            FILE(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/classes")
            ADD_CUSTOM_COMMAND(TARGET libtracijni POST_BUILD
                COMMAND "${Java_JAVAC_EXECUTABLE}" -d classes java/*.java
                COMMAND "${Java_JAR_EXECUTABLE}" -cfM ${CMAKE_SOURCE_DIR}/bin/libtracijni.jar -C classes .
                COMMAND "${Java_JAR_EXECUTABLE}" -cfM ${CMAKE_SOURCE_DIR}/bin/libtracijni_src.jar -C java .
            )
            set_property(TARGET libtracijni PROPERTY FOLDER "libtraci")
        else()
            message(WARNING "ENABLE_JAVA_BINDINGS is set but Java or JNI was not found.")
        endif()
    endif()

    find_package(PythonLibs)
    if(ENABLE_PYTHON_BINDINGS)
        if (MSVC)
            # recheck that the platform of the generator and python matches
            execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sys; print(sys.maxsize > 2**32)"
                            OUTPUT_VARIABLE IS_PYTHON64 OUTPUT_STRIP_TRAILING_WHITESPACE)
            if (${CMAKE_MODULE_LINKER_FLAGS} STREQUAL "/machine:x64")
                if (${IS_PYTHON64} STREQUAL "False")
                    message(STATUS "Did not find Python 64 bit. Please set PYTHON_EXECUTABLE, PYTHON_INCLUDE_DIR and PYTHON_LIBRARY manually.")
                    set(PYTHONLIBS_FOUND OFF)
                endif()
            else()
                if (${IS_PYTHON64} STREQUAL "True")
                    message(STATUS "Did not find Python 32 bit. Please set PYTHON_EXECUTABLE, PYTHON_INCLUDE_DIR and PYTHON_LIBRARY manually.")
                    set(PYTHONLIBS_FOUND OFF)
                endif()
            endif()
        endif()
        if(PYTHONLIBS_FOUND)
            if (PYTHON_DEBUG_LIBRARY OR NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
                include_directories(${PYTHON_INCLUDE_PATH})
#                set(CMAKE_SWIG_FLAGS "")
                set(CMAKE_SWIG_FLAGS "-keyword") # is necessary for using keyword args, but generates lots of warnings
# other options which should be evaluated: -builtin, -modern, -extranative
                set(CMAKE_SWIG_OUTDIR ${CMAKE_SOURCE_DIR}/tools/libtraci)
                set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/libtraci)
                set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/tools/libtraci)
                set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/tools/libtraci)
                set(SWIG_MODULE_libtraci_EXTRA_DEPS ${libtraci_HEADERS})
                if (${CMAKE_VERSION} VERSION_GREATER 3.8.0)
                    SWIG_ADD_LIBRARY(libtraci LANGUAGE python SOURCES libtraci.i)
                else()
                    SWIG_ADD_MODULE(libtraci python libtraci.i)
                endif()
                if (NOT DEFAULT_LIBSUMO_PYTHON)
                    # needed to selectively build the python module on the build server
                    set_property(TARGET ${SWIG_MODULE_libtraci_REAL_NAME} PROPERTY EXCLUDE_FROM_ALL TRUE)
                endif()
                if (MSVC OR APPLE)
                    # disable python module for the debug build because virtually no one has a python debug dll to link against
#                    set_property(TARGET ${SWIG_MODULE_libtraci_REAL_NAME} PROPERTY EXCLUDE_FROM_DEFAULT_BUILD_DEBUG TRUE)
                    set_property(TARGET ${SWIG_MODULE_libtraci_REAL_NAME} PROPERTY FOLDER "libtraci")
                    swig_link_libraries(libtraci libtracistatic ${libtcpip} ${PYTHON_LIBRARIES})
                else()
                    swig_link_libraries(libtraci -Wl,--whole-archive libtracistatic ${libtcpip} -Wl,--no-whole-archive)
                endif()
                if (NOT ${SETUPTOOLS_MISSING})
                    install(
                        CODE "execute_process(COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/build/setup-libtraci.py clean --all install --root=\$ENV{DESTDIR}/ --prefix=${CMAKE_INSTALL_PREFIX} --optimize=1)"
                        COMPONENT pylibtraci)
                endif()
            endif()
        else()
            message(WARNING "ENABLE_PYTHON_BINDINGS is set but Python libraries were not found.")
        endif()
    endif()

endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
add_library(libtracicpp SHARED ${libtracistatic_STAT_SRCS})
set_property(TARGET libtracicpp PROPERTY FOLDER "libtraci")
set_target_properties(libtracicpp PROPERTIES PREFIX "")
install(TARGETS libtracicpp DESTINATION ${CMAKE_INSTALL_LIBDIR})
if (MSVC OR APPLE)
    target_link_libraries(libtracicpp PRIVATE ${libtcpip})
else ()
    target_link_libraries(libtracicpp PRIVATE -Wl,--whole-archive ${libtcpip} -Wl,--no-whole-archive)
endif()
