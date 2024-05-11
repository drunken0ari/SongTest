add_executable(RANSAC
        SONG_RANSAC.cpp
        RANSAC_test.cpp)

target_link_libraries(RANSAC ${OpenCV_LIBS})