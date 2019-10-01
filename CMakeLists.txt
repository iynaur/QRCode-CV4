cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)

add_subdirectory(quirc)
project(qrcode)

file(GLOB QR_SRC "src/*.cpp")
file(GLOB QR_INC "src/*.h")

file(GLOB AT_SRC "apriltag/*.cpp")
file(GLOB AT_INC "apriltag/*.h")

source_group("QR Code" FILES ${QR_SRC} ${QR_INC})
source_group("AprilTag" FILES ${AT_SRC} ${AT_INC})

find_package (PCL 1.8 REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

#PCL
include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

find_package(OpenCV 4 REQUIRED)
include_directories(. quirc/include
  ${OpenCV_INCLUDE_DIRS}
  )
#link_directories(usr/local/opencv4/lib)
add_definitions(-DHAVE_OPENCV -DHAVE_QUIRC -DNOMINMAX)

find_package(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5Gui)

add_executable(qrcode ${QR_SRC} ${QR_INC} ${AT_SRC} ${AT_INC})
target_link_libraries(qrcode ${OpenCV_LIBS} quirc ${PCL_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  )
#target_link_libraries(qrcode
#  opencv_core
#  opencv_imgproc
#  )
