cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(Horn2VMT)
set(Horn2VMT_VERSION_MAJOR 1)
set(Horn2VMT_VERSION_MINOR 0)
set(Horn2VMT_VERSION ${Horn2VMT_VERSION_MAJOR}.${Horn2VMT_VERSION_MINOR})

# IN_LIST which FindDoxygen needs
cmake_policy(SET CMP0057 NEW)

#--------------------------------------------------------------------------------------------------
# Options:

option(ENABLE_ASSERTIONS "If false, -DNDEBUG is set" ON)
option(COMPILE_WITH_UBSAN "Compile with clang's UB sanitizer" OFF)
option(COMPILE_WITH_COVERAGE
  "Compile with clang's code coverage instrumentation" OFF)

#--------------------------------------------------------------------------------------------------
# Compiler settings:

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
    "Choose the type of build, options are: Debug RelWithDebInfo Release."
    FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Add path for custom modules
list (APPEND CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (COMPILE_WITH_UBSAN)
    add_compile_options(-fsanitize=undefined -fsanitize-recover=undefined
      -fno-omit-frame-pointer)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=undefined")
endif()

if (COMPILE_WITH_COVERAGE)
  add_compile_options(-fprofile-instr-generate -fcoverage-mapping)
  message(STATUS "Use LLVM_PROFILE_FILE=file ./horn2vmt to get profile info")
  if (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    message(WARNING "You are not compiling in Debug mode but you asked for 
    coverage compilation; in all likelihood, you want Debug mode, too")
  endif()
endif()

add_compile_options(-Wall -Wextra -Werror -Wno-unused-function -Wunused-variable
  -Wno-reorder -fno-omit-frame-pointer -g)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
  add_compile_options( -Weffc++ -Wheader-hygiene -Wold-style-cast)
endif()

message(STATUS "Compiler ID is " "${CMAKE_CXX_COMPILER_ID}")

# set actual compilation flags based on options
set(CMAKE_CXX_FLAGS_DEBUG "-O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")

#--------------------------------------------------------------------------------------------------
# Dependencies:

#set(CMAKE_THREAD_PREFER_PTHREAD ON)
#include(FindThreads)

set(FMT_DIR ${CMAKE_SOURCE_DIR}/lib/fmt)
add_subdirectory(${FMT_DIR} EXCLUDE_FROM_ALL)
include_directories(SYSTEM ${FMT_DIR}/include)

# assumes Z3_DIR/build is where z3 dylib is
if(NOT DEFINED Z3_DIR)
  set(Z3_DIR /opt/z3-4.6.0/)
endif()
if(EXISTS "${Z3_DIR}" AND IS_DIRECTORY "${Z3_DIR}")
  message(STATUS "Using z3 at: ${Z3_DIR}")
else()
  message(FATAL_ERROR "Z3_DIR not found: ${Z3_DIR}")
  return()
endif()
include_directories(SYSTEM ${Z3_DIR}/include)
link_directories(${Z3_DIR}/lib)

# Path to LLVM install
if(NOT DEFINED LLVM_DIR)
  set(LLVM_DIR /opt/llvm-5.0.1/lib/cmake/llvm)
endif()
find_package(LLVM CONFIG NO_DEFAULT_PATH)
if (NOT LLVM_FOUND)
  message(FATAL_ERROR "No llvm found. Please find path/to/cmake/llvm.")
  return()
    #else()
    #set_target_properties(llvm PROPERTIES EXCLUDE_FROM_ALL ON)
endif()

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
message(STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")

include_directories(SYSTEM ${LLVM_INCLUDE_DIRS})
add_definitions(${LLVM_DEFINITIONS})

#find_package(Gmp REQUIRED)
#if (GMP_FOUND)
#  include_directories (${GMP_INCLUDE_DIR})
#  include_directories (${GMPXX_INCLUDE_DIR})
#else()
#  message(FATAL_ERROR "No gmp found")
#  return()
#  # set(GMP_LIB "")
#  # set(GMPXX_LIB "")
#endif()

find_package(Boost
    1.67.0 REQUIRED 
    OPTIONAL_COMPONENTS program_options)
if (Boost_FOUND)
  include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Boost 1.67.0 required")
endif()
if(NOT ${Boost_PROGRAM_OPTIONS_FOUND})
    message(FATAL_ERROR "Boost program_options required but not found")
endif()

#include_directories(src)

#--------------------------------------------------------------------------------------------------
# Compile flags:

if (NOT ENABLE_ASSERTIONS)
  add_definitions(-DNDEBUG)
endif()

#--------------------------------------------------------------------------------------------------
# Other stuff:

#--------------------------------------------------------------------------------------------------
# Build targets:

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
add_custom_target(make-bin-directory ALL COMMAND ${CMAKE_COMMAND}
  -E make_directory ${CMAKE_BINARY_DIR}/bin)
llvm_map_components_to_libnames(llvm_libs support)

add_executable(horn2vmt tools/horn2vmt/main.cc)
target_link_libraries(horn2vmt
    Boost::program_options
    ${llvm_libs} z3 ${GMPXX_LIB} ${GMP_LIB} fmt)

if (COMPILE_WITH_COVERAGE)
  target_link_options(horn2vmt BEFORE
    PUBLIC -fprofile-instr-generate -fcoverage-mapping)
endif()

install(TARGETS horn2vmt
  RUNTIME DESTINATION bin)

add_dependencies(horn2vmt make-bin-directory)

add_custom_target(tags
  COMMAND ctags --c++-kinds=+p --c-kinds=+p --fields=+iaS --extra=+q -Rnu
  ./src ./tools ${Z3_DIR}/include ${BOOLECTOR_DIR}/src/boolector.h
  ${MATHSAT_DIR}/include ${LLVM_INCLUDE_DIRS}
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

function(add_clang_static_analysis target)
  get_target_property(SRCs ${target} SOURCES)
  add_library(${target}_analyze OBJECT EXCLUDE_FROM_ALL ${SRCs})
  set_target_properties(${target}_analyze PROPERTIES
    COMPILE_OPTIONS "--analyze;--analyzer-output;html"
    EXCLUDE_FROM_DEFAULT_BUILD true)
endfunction()
add_clang_static_analysis(horn2vmt)
