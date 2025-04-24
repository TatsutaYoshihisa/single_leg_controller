cmake_minimum_required(VERSION 3.0.2)
project(single_leg_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  dynamixel_sdk
)

# メッセージファイルの追加
add_message_files(
  FILES
  LegCommand.msg
  LegPosition.msg
)

# メッセージの生成
generate_messages(
  DEPENDENCIES
  std_msgs
)

# パッケージの設定
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime dynamixel_sdk
)

# インクルードディレクトリの設定
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##-----------------------------------<ver1>------------------------------
# 実行ファイルの生成
add_executable(${PROJECT_NAME}_node 
src/ver1_single_leg_controller.cpp
src/ver1_dynamixel_sync_controller.cpp
)
# 依存関係の設定
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
# リンクの設定
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

##-----------------------------------<ver2>------------------------------
# # 実行ファイルの生成
# add_executable(${PROJECT_NAME}_node 
# src/ver2_single_leg_controller.cpp
# src/ver1_dynamixel_sync_controller.cpp
# )
# # 依存関係の設定
# add_dependencies(${PROJECT_NAME}_node 
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#   ${catkin_EXPORTED_TARGETS}
# )
# # リンクの設定
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
