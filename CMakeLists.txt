add_library(data_recorder SHARED data_recorder.cpp)

target_include_directories(data_recorder PUBLIC
  ${PROJECT_SOURCE_DIR}/data_recorder
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)