cmake_minimum_required(VERSION 3.16)
project(control_system_design_guide_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(MSVC)
  add_compile_options(/W4 /utf-8)
else()
  add_compile_options(-Wall -Wextra)
endif()

enable_testing()

# 每章一個執行檔:ch01 … ch19;ctest 執行時 CSV 輸出到 build/out/
file(GLOB CHAPTER_DIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/[01][0-9]-*)
list(SORT CHAPTER_DIRS)
foreach(dir ${CHAPTER_DIRS})
  string(SUBSTRING ${dir} 0 2 num)
  add_executable(ch${num} ${dir}/main.cpp)
  add_test(NAME ${dir} COMMAND ch${num})
  set_tests_properties(${dir} PROPERTIES ENVIRONMENT "CSD_OUT=${CMAKE_BINARY_DIR}/out")
endforeach()

# 韌體 PID 的 host 端測試(與 hardware/esp32/test_host 相同來源)
add_executable(test_pid ${CMAKE_CURRENT_SOURCE_DIR}/../hardware/esp32/test_host/test_pid.cpp)
