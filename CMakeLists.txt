cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(MultiKinectViewer CXX) # change project(pcl_visualizer_viewports)

find_package(PCL 1.2 REQUIRED)

#set(INCLUDE_DIR ../MultiKinectViewer)
include_directories (${INCLUDE_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})





#file(GLOB_RECURSE Foo_SOURCES "*.cpp")
#file(GLOB_RECURSE Foo_HEADERS "*.h")

#set (Foo_INCLUDE_DIRS "")
#foreach (_headerFile ${Foo_HEADERS})
#    get_filename_component(_dir ${_headerFile} PATH)
#    list (APPEND Foo_INCLUDE_DIRS ${_dir})
#endforeach()
#list(REMOVE_DUPLICATES Foo_INCLUDE_DIRS)

#include_directories(${Foo_INCLUDE_DIRS})
set (CMAKE_CXX_STANDARD 11)

macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)
#add_executable (FooExe ${Foo_SOURCES})

#add_executable (MultiKinectViewer MultiKinectViewer.cpp) #change _demo
#target_link_libraries (MultiKinectViewer ${PCL_LIBRARIES}) #change _demo

#add_executable (MultiKinectThreading MultiKinectThreading.cpp) #change _demo
#target_link_libraries (MultiKinectThreading ${PCL_LIBRARIES}) #change _demo

#add_executable (MultiKinectPthread MultiKinectPthread.cpp) #change _demo
#target_link_libraries (MultiKinectPthread ${PCL_LIBRARIES}) #change _demo

#add_executable (MultiKinectThreadingSave MultiKinectThreadingSave.cpp) #change _demo
#target_link_libraries (MultiKinectThreadingSave ${PCL_LIBRARIES}) #change _demo

#add_executable (MultiKinectThreadingSaveGen ${Foo_SOURCES} ) #change _demo
#target_link_libraries (MultiKinectThreadingSaveGen ${PCL_LIBRARIES} ) #change _demo

#add_executable (MultiKinectThreadingSaveGen MultiKinectThreadingSaveGen FuncReg.cpp FuncReg.h ) #change _demo
#target_link_libraries (MultiKinectThreadingSaveGen ${PCL_LIBRARIES} ) #change _demo

add_executable (MultiKinect MultiKinect.cpp FuncReg.cpp FuncReg.h ) #change _demo
target_link_libraries (MultiKinect ${PCL_LIBRARIES} ) #change _demo






