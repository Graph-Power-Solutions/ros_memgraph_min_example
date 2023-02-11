# Ubuntu 22.04 LTS ARM

## Build from source
```
git clone git@github.com:memgraph/memgraph.git
cd memgraph/
mkdir build
cd build/
cmake ..

sudo apt-get install python3-dev cmake make gcc g++ libssl-dev

wget https://boostorg.jfrog.io/artifactory/main/release/1.78.0/source/boost_1_78_0.tar.gz
tar xzvf boost_1_78_0.tar.gz
./bootstrap.sh --prefix=/usr/
./b2
sudo ./b2 install

sudo apt-get install libbz2-dev
# libbz2-dev:arm64 (1.0.8-5build1)

sudo apt-get install libgflags-dev
# libgflags-dev (2.2.2-2)

sudo apt-get install libfmt-dev
# libfmt-dev:arm64 (8.1.1+ds1-2)

sudo apt-get install libjemalloc-dev
# libjemalloc-dev (5.2.1-4ubuntu1)
```

```
-- CCache: ON
-- Memgraph version: 2.5.0+4~beaba0fc1
-- Memgraph DEB version: 2.5.0+4~beaba0fc1-1
-- Memgraph RPM version: 2.5.0_0.4.beaba0fc1
-- SOURCE_DIR/home/alex/memgraph
-- BINARY_DIR/home/alex/memgraph/build
-- CMake build type: Debug
-- Could not find Readline
-- Found Jemalloc: /usr/lib/aarch64-linux-gnu/libjemalloc.a  
-- Found ZLIB: /usr/lib/aarch64-linux-gnu/libz.so (found suitable version "1.2.11", minimum required is "1.2.11") 
CMake Error at libs/CMakeLists.txt:127 (add_subdirectory):
  add_subdirectory given source "rapidcheck" which is not an existing
  directory.


-- Found OpenSSL: /usr/lib/aarch64-linux-gnu/libcrypto.so (found version "3.0.2")  
CMake Error at /usr/share/cmake-3.22/Modules/ExternalProject.cmake:2882 (message):
  No download info given for 'spdlog-populate' and its source directory:

   /home/alex/memgraph/libs/spdlog

  is not an existing non-empty directory.  Please specify one of:

   * SOURCE_DIR with an existing non-empty directory
   * DOWNLOAD_COMMAND
   * URL
   * GIT_REPOSITORY
   * SVN_REPOSITORY
   * HG_REPOSITORY
   * CVS_REPOSITORY and CVS_MODULE
Call Stack (most recent call first):
  /usr/share/cmake-3.22/Modules/ExternalProject.cmake:3716 (_ep_add_download_command)
  CMakeLists.txt:15 (ExternalProject_Add)


-- Configuring incomplete, errors occurred!
See also "/home/alex/memgraph/build/_deps/spdlog-subbuild/CMakeFiles/CMakeOutput.log".

CMake Error at /usr/share/cmake-3.22/Modules/FetchContent.cmake:1075 (message):
  CMake step for spdlog failed: 1
Call Stack (most recent call first):
  /usr/share/cmake-3.22/Modules/FetchContent.cmake:1216:EVAL:2 (__FetchContent_directPopulate)
  /usr/share/cmake-3.22/Modules/FetchContent.cmake:1216 (cmake_language)
  /usr/share/cmake-3.22/Modules/FetchContent.cmake:1259 (FetchContent_Populate)
  libs/CMakeLists.txt:198 (FetchContent_MakeAvailable)


-- Configuring incomplete, errors occurred!
See also "/home/alex/memgraph/build/CMakeFiles/CMakeOutput.log".

```

## Package .deb
```
wget https://download.memgraph.com/memgraph/v2.5.2/debian-11-aarch64/memgraph_2.5.2-1_arm64.deb

sudo dpkg -i memgraph_2.5.2-1_arm64.deb

Selecting previously unselected package memgraph.
(Reading database ... 99756 files and directories currently installed.)
Preparing to unpack memgraph_2.5.2-1_arm64.deb ...
Unpacking memgraph (2.5.2-1) ...
dpkg: dependency problems prevent configuration of memgraph:
 memgraph depends on libpython3.9 (>= 3.9.1); however:
  Package libpython3.9 is not installed.
 memgraph depends on libssl1.1 (>= 1.1.0); however:
  Package libssl1.1 is not installed.
```

