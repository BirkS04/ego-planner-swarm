cmake_minimum_required(VERSION 3.8)
project(lkh_tsp_solver)

# HIER IST DER FIX: -fcommon für den C- und C++ Compiler
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcommon")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcommon")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -O3 -w) # -w schaltet lästige C-Warnungen ab
endif()

find_package(ament_cmake REQUIRED)

include_directories(include)

# Sammle alle .c und .cpp Dateien
file(GLOB SRC_FILES "src/*.c" "src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()