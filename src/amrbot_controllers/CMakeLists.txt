cmake_minimum_required(VERSION 3.8)
project(amrbot_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py" PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
