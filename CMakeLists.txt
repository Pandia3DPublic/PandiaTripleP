cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)
cmake_policy(SET CMP0015 NEW)

project(StaticScanner LANGUAGES CXX)

macro(print_all_variables)
    message(STATUS "print_all_variables------------------------------------------{")
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "print_all_variables------------------------------------------}")
endmacro()

file(GLOB_RECURSE srcfiles 
src/*.h   
src/*.hpp
src/*.cpp
3rdParty/SickCamera/*.hpp
3rdParty/SickCamera/*.h
3rdParty/SickCamera/*.cpp
)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/3rdParty/SickCamera/)
include_directories(${PROJECT_SOURCE_DIR}/3rdParty/SickCamera/3rdParty)
include_directories(SYSTEM	${PROJECT_SOURCE_DIR}/pchs)

#find lib packages here
set(Open3D_DIR ${PROJECT_SOURCE_DIR}/3rdParty/Open3D/install/lib/cmake/Open3D)
find_package(Open3D HINTS Open3D_DIR REQUIRED)
set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/3rdParty/opencv/lib/cmake/opencv4/)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(pybind11 REQUIRED HINTS ${PROJECT_SOURCE_DIR}/3rdParty/pybind11/share/cmake)
find_package(jsoncpp REQUIRED HINTS ${PROJECT_SOURCE_DIR}/3rdParty/jsoncpp/lib/cmake)
get_target_property(JSONCPP_INCLUDE_DIRS jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
# camera packages
find_package(k4a 1.4.1 REQUIRED)
find_package(realsense2 2.50.0 REQUIRED)
find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
set(Phoxi_Version 1.9.0)
if (NOT ${PHO_SOFTWARE_VERSION} MATCHES ${Phoxi_Version})
    message( FATAL_ERROR "Installed PhoXi version is ${PHO_SOFTWARE_VERSION} but expected ${Phoxi_Version}" )
endif()
find_package(Zivid 2.7.0 COMPONENTS Core REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{ENSENSO_INSTALL}/cmake/")
find_package(Ensenso REQUIRED)
set(Ensenso_Version 3.3.1385)
if (NOT ${PC_ENSENSO_VERSION} MATCHES ${Ensenso_Version})
    message( FATAL_ERROR "Installed Ensenso version is ${PC_ENSENSO_VERSION} but expected ${Ensenso_Version}" )
endif()
# print_all_variables()

#choose targets
if (${EXECUTABLE} STREQUAL "Test") #set in runcmake
    message("################### Building Test ###################")
    set(target_name Test) #cpp files with same name must be located in /exe 
    add_executable(${target_name} exe/${target_name}.cpp)
endif()
if (${EXECUTABLE} STREQUAL "cppBindings")
    message("################### Building cppBindings ###################")
    set(target_name cppBindings) #cpp files with same name must be located in /exe 
    pybind11_add_module(${target_name} src/cppBindings/cppBindings.cpp)
endif()

#our 3rd party stuff
file(COPY 3rdParty/Open3D/install/lib/libOpen3D.so DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/cppLibs)
file(COPY 3rdParty/jsoncpp/lib/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/cppLibs)
file(COPY 3rdParty/opencv/lib/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/cppLibs/opencv)

set_target_properties(${target_name} PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(${target_name} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
set_target_properties(${target_name} PROPERTIES INSTALL_RPATH "$ORIGIN/;$ORIGIN/cppLibs;$ORIGIN/cppLibs/opencv;")

target_sources(${target_name} PRIVATE ${srcfiles})
target_precompile_headers(${target_name} PUBLIC ${PROJECT_SOURCE_DIR}/pchs/pch.h)

target_link_libraries(${target_name} PRIVATE k4a)
target_include_directories(${target_name} PUBLIC ${Open3D_INCLUDE_DIRS})
target_link_libraries(${target_name} PRIVATE ${Open3D_LIBRARIES})
target_include_directories(${target_name} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${target_name} PRIVATE ${OpenCV_LIBRARIES})
target_link_libraries(${target_name} PRIVATE ${realsense2_LIBRARY})
target_link_libraries(${target_name} PRIVATE OpenMP::OpenMP_CXX)
target_include_directories (${target_name} PUBLIC ${PHOXI_INCLUDE_DIRS})
target_link_libraries(${target_name} PRIVATE ${PHOXI_LIBRARY} $<$<PLATFORM_ID:Linux>:rt>)
target_include_directories (${target_name} PUBLIC ${pybind11_INCLUDE_DIRS})
target_link_libraries(${target_name} PRIVATE pybind11::pybind11 pybind11::embed pybind11::module) #gruselig TODO
target_include_directories (${target_name} PUBLIC ${ZIVID_INCLUDE_DIRS})
target_link_libraries(${target_name} PRIVATE Zivid::Core)
target_include_directories (${target_name} PUBLIC ${ENSENSO_INCLUDE_DIRS})
target_link_libraries(${target_name} PRIVATE ${ENSENSO_LIBRARIES})
target_include_directories (${target_name} PUBLIC ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(${target_name} PRIVATE jsoncpp_lib)
# target_compile_definitions(${target_name} PRIVATE ${compileDefs})

