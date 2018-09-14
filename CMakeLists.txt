cmake_minimum_required(VERSION 2.8.3)
project(kn_wheel_group)

###################################
## catkin                        ##
###################################
find_package(catkin  REQUIRED COMPONENTS
  irg_cmake
  Miro
  kn
  knDds
  knRapid
  roscpp
  qt_gui_cpp
)
include_directories( ${CATKIN_DEVEL_PREFIX}/include )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  message(STATUS "FOUND QT 5")
  find_package(Qt5Core REQUIRED)
  find_package(Qt5Xml REQUIRED)
  if (Qt5_POSITION_INDEPENDENT_CODE)
    SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
  endif()
  include_directories(SYSTEM "${Qt5Core_INCLUDE_DIRS}")
  include_directories(SYSTEM "${Qt5Xml_INCLUDE_DIRS}")
  set(QT_QTXML_LIBRARY Qt5::Xml)
  set(QT_QTCORE_LIBRARY Qt5::Widgets)
  set( qt_LIBRARIES    ${Qt5Core_LIBRARIES}    ${Qt5Xml_LIBRARIES} )
else()
  message(STATUS "FOUND QT 4")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
  include_directories(SYSTEM "${QT_INCLUDE_DIR}")
  include_directories(SYSTEM "${QT_QTXML_INCLUDE_DIR}")
  include_directories(SYSTEM "${QT_QTCORE_INCLUDE_DIR}")
endif()

find_package( ACE )
find_package( Boost 1.50.0 COMPONENTS program_options filesystem system chrono)
find_package( Eigen3 REQUIRED)
find_package( RtiDds  REQUIRED)
find_package( Doxygen )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES knMotor knMotorImpl knProtoMotor knSimMotor
  CATKIN_DEPENDS kn Miro
  CFG_EXTRAS
  DEPENDS
)

# add devel/include - this should add to persistent catkin metadata
list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)

## We must add the in-source and out-of-source
## include paths
##--------------------------------------------
include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_BINARY_DIR}/src
)

include_directories( ${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include )
include_directories( SYSTEM
  ${EIGEN3_INCLUDE_DIR}
)
include_directories(SYSTEM
  ${Boost_INCLUDE_DIR}
)

## Set up RPATH and *Config.h
## Call these after we've made all our
## find_package calls
##--------------------------------------------
setup_rpath()
#setup_configure_file()

#-----------------------------------
add_subdirectory( src )

