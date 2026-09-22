cmake_minimum_required(VERSION 3.10)
project(CurrentSystem)

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(CurrentSystem SHARED src/CurrentSystem.cc)

target_link_libraries(CurrentSystem
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS CurrentSystem
  LIBRARY DESTINATION lib
)
