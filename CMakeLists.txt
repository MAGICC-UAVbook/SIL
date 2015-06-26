cmake_minimum_required(VERSION 2.8.3)
project(sil_mex)

# add package modules path, not needed in dependend packages
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
include(EnableCXX11)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules) # add FindMatlab module

add_definitions(/DMATLAB_MEX_FILE) #define matlab macros
add_definitions(/DMX_COMPAT_32)

find_package(Matlab REQUIRED)

IF(MATLAB_FOUND)
    message(STATUS "MATLAB Found, MATLAB MEX will be compiled.")
ELSE(MATLAB_FOUND)
    MESSAGE("MATLAB not found...nothing will be built.")
ENDIF(MATLAB_FOUND)

include_directories(
  include
)

# compile mex
set(CPP_FILE SIL)

# set up matlab libraries
set(CPP_FILE_NAME src/${CPP_FILE}.cpp)
INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR} ${SIMULINK_INCLUDE_DIR})
add_library(${CPP_FILE} SHARED ${CPP_FILE_NAME} ${CMAKE_SOURCE_DIR}/Matlabdef.def
  src/lib/controller_base.cpp
  src/lib/controller_example.cpp
  include/controller_base.h
  include/controller_example.h
  )

target_link_libraries(${CPP_FILE} ${MATLAB_LIBRARIES}
)

# 32-bit or 64-bit mex
if(WIN32)
  if (CMAKE_CL_64)
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexw64)
  else(CMAKE_CL_64)
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexw32)
  endif(CMAKE_CL_64)
else(WIN32)
  if (CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexa64 PREFIX "")
  else(CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexglx PREFIX "")
  endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
endif(WIN32)
