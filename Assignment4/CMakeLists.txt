find_package(Pangolin REQUIRED)

add_executable(PlaneFitting PlaneFitting.cpp)

target_link_libraries(PlaneFitting ${OpenCV_LIBS} ${Pangolin_LIBRARIES})