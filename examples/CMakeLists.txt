# サンプルプログラム用CMakeLists.txt

# PID制御サンプル
add_executable(pid_example pid_example.cpp)
target_link_libraries(pid_example PRIVATE omuraisu_pid)

# メカナムホイールサンプル
add_executable(mecanum_example mecanum_example.cpp)
target_link_libraries(mecanum_example PRIVATE omuraisu_chassis)

# 座標演算サンプル
add_executable(coordinate_example coordinate_example.cpp)
target_link_libraries(coordinate_example PRIVATE coordinate)
