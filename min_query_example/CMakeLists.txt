cmake_minimum_required(VERSION 3.8)
project(ros_memgraph_min_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ## Memgraph mgclient clone library https://github.com/memgraph/mgclient ###
include(ExternalProject)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(OpenSSL REQUIRED)

set(MGCLIENT_GIT_TAG "v1.3.0" CACHE STRING "mgclient git tag")
set(MGCLIENT_LIBRARY mgclient-lib)
set(MGCLIENT_INSTALL_DIR ${CMAKE_BINARY_DIR}/mgclient)
set(MGCLIENT_INCLUDE_DIRS ${MGCLIENT_INSTALL_DIR}/include)

if(UNIX AND NOT APPLE)
  set(MGCLIENT_LIBRARY_PATH ${MGCLIENT_INSTALL_DIR}/lib/libmgclient.so)
elseif(WIN32)
  set(MGCLIENT_LIBRARY_PATH ${MGCLIENT_INSTALL_DIR}/lib/mgclient.dll)
endif()

ExternalProject_Add(mgclient-proj
  PREFIX mgclient-proj
  GIT_REPOSITORY https://github.com/memgraph/mgclient.git
  GIT_TAG "${MGCLIENT_GIT_TAG}"
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${MGCLIENT_INSTALL_DIR}"
  "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
  "-DBUILD_CPP_BINDINGS=ON"
  "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
  BUILD_BYPRODUCTS "${MGCLIENT_LIBRARY_PATH}"
  INSTALL_DIR "${PROJECT_BINARY_DIR}/mgclient"
)
add_library(${MGCLIENT_LIBRARY} SHARED IMPORTED)
target_compile_definitions(${MGCLIENT_LIBRARY} INTERFACE mgclient_shared_EXPORTS)
set_property(TARGET ${MGCLIENT_LIBRARY} PROPERTY IMPORTED_LOCATION ${MGCLIENT_LIBRARY_PATH})

if(WIN32)
  set_property(TARGET ${MGCLIENT_LIBRARY} PROPERTY IMPORTED_IMPLIB ${MGCLIENT_INSTALL_DIR}/lib/mgclient.lib)
endif()

add_dependencies(${MGCLIENT_LIBRARY} mgclient-proj)

# ## Memgraph clone end ###

# Executable
add_executable(query src/query.cpp)
ament_target_dependencies(query rclcpp)

# ## Memgraph add_dependencies ###
add_dependencies(query ${MGCLIENT_LIBRARY})
target_link_libraries(query
  ${catkin_LIBRARIES}
  ${MGCLIENT_LIBRARY}
)

target_include_directories(query PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PROJECT_NAME} PRIVATE ${MGCLIENT_INCLUDE_DIRS}) # ## Memgraph libraries path ###
target_compile_features(query PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

install(TARGETS query
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
