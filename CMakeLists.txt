cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

project( kinect_grabber_windows )


# Find Packages
find_package(OpenCV REQUIRED )
find_package( PCL 1.8 REQUIRED )
find_package( KinectSDK2 REQUIRED )
find_package(Qt5 REQUIRED Widgets)
find_package(VTK REQUIRED)

if( PCL_FOUND AND KinectSDK2_FOUND AND Qt5_FOUND AND VTK_FOUND)
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )
  include_directories(${OpenCV_INCLUDE_DIRS})
  include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

  # Preprocessor Definitions
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )

endif()


add_executable( kinect_grabber_windows kinect2_grabber.h main.cpp )
# Additional Dependencies
target_link_libraries( kinect_grabber_windows ${PCL_LIBRARIES})
target_link_libraries( kinect_grabber_windows ${KinectSDK2_LIBRARIES} ${OpenCV_LIBS})