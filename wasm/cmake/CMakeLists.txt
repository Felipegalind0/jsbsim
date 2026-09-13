cmake_minimum_required(VERSION 3.20)

project(jsbsim_wasm LANGUAGES CXX)

if(NOT JSBSIM_SOURCE_DIR)
  message(FATAL_ERROR "JSBSIM_SOURCE_DIR is required")
endif()

foreach(required JSBSIM_WASM_BINDINGS JSBSIM_BINDINGS_MANIFEST JSBSIM_WASM_EXTENSION_DIR JSBSIM_BUILD_INPUT_SHA256 JSBSIM_NATIVE_CONTENT_SHA256)
  if(NOT ${required})
    message(FATAL_ERROR "${required} must come from the resolved build descriptor")
  endif()
endforeach()
file(READ "${JSBSIM_BINDINGS_MANIFEST}" BINDINGS_JSON)
string(JSON BINDINGS_SOURCE GET "${BINDINGS_JSON}" nativeContentSha256)
string(JSON BINDINGS_INPUT GET "${BINDINGS_JSON}" inputSha256)
string(JSON BINDINGS_SHA GET "${BINDINGS_JSON}" files "generated/FGFDMExecBindings.cpp")
file(SHA256 "${JSBSIM_WASM_BINDINGS}" ACTUAL_BINDINGS_SHA)
if(NOT BINDINGS_SOURCE STREQUAL JSBSIM_NATIVE_CONTENT_SHA256 OR NOT BINDINGS_INPUT STREQUAL JSBSIM_BUILD_INPUT_SHA256 OR NOT BINDINGS_SHA STREQUAL ACTUAL_BINDINGS_SHA)
  message(FATAL_ERROR "Native source and generated bindings identity mismatch")
endif()

if(NOT EXISTS "${JSBSIM_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "JSBSim source not found at ${JSBSIM_SOURCE_DIR}")
endif()

if(NOT EXISTS "${JSBSIM_WASM_BINDINGS}")
  message(FATAL_ERROR "Generated bindings not found at ${JSBSIM_WASM_BINDINGS}")
endif()

set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_JULIA_PACKAGE OFF CACHE BOOL "" FORCE)
set(BUILD_MATLAB_SFUNCTION OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(SYSTEM_EXPAT OFF CACHE BOOL "" FORCE)

add_subdirectory("${JSBSIM_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/jsbsim" EXCLUDE_FROM_ALL)

# Hand-written extensions live beside the generated FGFDMExec bindings.
file(GLOB JSBSIM_WASM_EXTENSION_BINDINGS CONFIGURE_DEPENDS "${JSBSIM_WASM_EXTENSION_DIR}/*.cpp")

add_executable(jsbsim_wasm "${JSBSIM_WASM_BINDINGS}" ${JSBSIM_WASM_EXTENSION_BINDINGS})

set_target_properties(jsbsim_wasm PROPERTIES
  OUTPUT_NAME "jsbsim_wasm"
  SUFFIX ".mjs"
)

target_compile_features(jsbsim_wasm PRIVATE cxx_std_17)
target_compile_options(jsbsim_wasm PRIVATE -fexceptions)

target_include_directories(jsbsim_wasm PRIVATE
  "${JSBSIM_SOURCE_DIR}/src"
)

target_link_libraries(jsbsim_wasm PRIVATE libJSBSim)

target_link_options(jsbsim_wasm PRIVATE
  --bind
  -O3
  -sMODULARIZE=1
  -sEXPORT_ES6=1
  -sEXPORT_NAME=JSBSimWasmModule
  -sENVIRONMENT=web,worker,node
  -sFILESYSTEM=1
  -sALLOW_MEMORY_GROWTH=1
  -sNO_DISABLE_EXCEPTION_CATCHING
  -sWASM_BIGINT=1
  "-sEXPORTED_RUNTIME_METHODS=['FS']"
)
