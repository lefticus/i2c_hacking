# Generic test that uses conan libs
add_executable(i2c_hacking main.cpp)
target_link_libraries(
  i2c_hacking
  PRIVATE i2c
          project_options
          project_warnings
          CONAN_PKG::docopt.cpp
          CONAN_PKG::fmt
          CONAN_PKG::spdlog)
