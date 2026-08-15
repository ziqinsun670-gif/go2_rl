cmake_minimum_required(VERSION 3.16)
project(go2_policy_benchmark LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ORT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/onnxruntime")
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ORT_ROOT}/lib/libonnxruntime.so.1.22.0"
    INTERFACE_INCLUDE_DIRECTORIES "${ORT_ROOT}/include"
)

add_executable(go2_policy_benchmark policy_benchmark.cpp)
target_link_libraries(go2_policy_benchmark PRIVATE onnxruntime)
set_target_properties(go2_policy_benchmark PROPERTIES
    BUILD_RPATH "${ORT_ROOT}/lib"
)

set(UNITREE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_sdk2" CACHE PATH
    "Path to the Unitree SDK2 source tree")
set(UNITREE_SDK_LIB_DIR "${UNITREE_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}")
set(UNITREE_DDS_LIB_DIR "${UNITREE_SDK_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")

if(EXISTS "${UNITREE_SDK_LIB_DIR}/libunitree_sdk2.a"
   AND EXISTS "${UNITREE_DDS_LIB_DIR}/libddsc.so"
   AND EXISTS "${UNITREE_DDS_LIB_DIR}/libddscxx.so")
    find_package(Threads REQUIRED)

    add_library(unitree_sdk2 STATIC IMPORTED)
    set_target_properties(unitree_sdk2 PROPERTIES
        IMPORTED_LOCATION "${UNITREE_SDK_LIB_DIR}/libunitree_sdk2.a"
        INTERFACE_INCLUDE_DIRECTORIES "${UNITREE_SDK_ROOT}/include"
    )

    add_library(unitree_ddsc SHARED IMPORTED)
    set_target_properties(unitree_ddsc PROPERTIES
        IMPORTED_LOCATION "${UNITREE_DDS_LIB_DIR}/libddsc.so"
        IMPORTED_NO_SONAME TRUE
        INTERFACE_INCLUDE_DIRECTORIES "${UNITREE_SDK_ROOT}/thirdparty/include"
    )

    add_library(unitree_ddscxx SHARED IMPORTED)
    set_target_properties(unitree_ddscxx PROPERTIES
        IMPORTED_LOCATION "${UNITREE_DDS_LIB_DIR}/libddscxx.so"
        IMPORTED_NO_SONAME TRUE
        INTERFACE_INCLUDE_DIRECTORIES "${UNITREE_SDK_ROOT}/thirdparty/include;${UNITREE_SDK_ROOT}/thirdparty/include/ddscxx"
    )

    add_executable(go2_shadow_inference go2_shadow_inference.cpp)
    target_link_libraries(go2_shadow_inference PRIVATE
        onnxruntime
        unitree_sdk2
        unitree_ddsc
        unitree_ddscxx
        Threads::Threads
    )
    target_link_options(go2_shadow_inference PRIVATE "-Wl,--disable-new-dtags")
    set_target_properties(go2_shadow_inference PROPERTIES
        BUILD_RPATH "${ORT_ROOT}/lib;${UNITREE_DDS_LIB_DIR}"
    )
else()
    message(STATUS "Unitree SDK2 not found; go2_shadow_inference will not be built")
endif()
