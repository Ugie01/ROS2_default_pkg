# CMake 최소 버전 요구 (3.8 이상)
cmake_minimum_required(VERSION 3.8)

# 프로젝트 이름 설정 (패키지 이름과 동일하게 유지)
project(node_default)

# GCC/Clang 컴파일러 경고 옵션 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)  # 모든 경고 활성화
endif()

# ROS2 의존성 패키지 검색
find_package(ament_cmake REQUIRED)  # ROS2 빌드 시스템
find_package(rclcpp REQUIRED)       # ROS2 C++ 클라이언트 라이브러리
find_package(std_msgs REQUIRED)     # 표준 메시지 패키지

# node_1 실행파일 빌드 설정
add_executable(node_1 src/node_1.cpp)
# 헤더 파일 경로 설정 (빌드/설치 환경 모두 적용)
target_include_directories(node_1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# ROS2 의존성 연결
ament_target_dependencies(node_1 rclcpp std_msgs)

# node_2 실행파일 빌드 설정 (node_1과 동일 구조)
add_executable(node_2 src/node_2.cpp)
target_include_directories(node_2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(node_2 rclcpp std_msgs)

# 빌드 결과물 설치 경로 설정
install(TARGETS
  node_1
  node_2
  DESTINATION lib/${PROJECT_NAME}  # 설치 경로: ~/install/lib/node_default
)

# 테스트 설정 (기본적으로 비활성화)
if(BUILD_TESTING)
  # 자동 코드 검사 도구 설정
  find_package(ament_lint_auto REQUIRED)
  # 저작권 검사 비활성화 (임시 설정)
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint 검사 비활성화 (git 저장소 없을 때)
  set(ament_cmake_cpplint_FOUND TRUE)
  # 테스트 의존성 자동 검색
  ament_lint_auto_find_test_dependencies()
endif()

# ROS2 패키지 메타정보 생성 및 설치 설정
ament_package()