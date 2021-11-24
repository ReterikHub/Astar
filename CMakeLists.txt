cmake_minimum_required (VERSION 3.0.0)
project (Astar)

include_directories ("./" "${PROJECT_BINARY_DIR}")

link_directories("freeglut")
link_directories("freeglut/build/lib/Debug")
add_executable (Astar Astar.cpp)
target_link_libraries (Astar ${EXTRA_LIBS})	 