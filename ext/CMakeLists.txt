cmake_minimum_required(VERSION 3.1)

add_subdirectory(nanoplan)

include(FetchContent)
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 7.1.3
)
FetchContent_MakeAvailable(fmt)

