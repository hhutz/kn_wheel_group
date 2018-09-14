#
# Build knUrgLidar library
#

set( LIB_NAME    "knSimMotor" )
set( DIR_NAME    "knSimMotor" )
set( EXPORT      "knSimMotor" )
set( EXPORT_NAME "${EXPORT}_Export" )
set( EXPORT_FILE "${EXPORT_NAME}.h" )
string( TOUPPER  "${EXPORT}_BUILD_DLL" EXPORT_DEFINE )

## Miro parameters
##------------------------------------------------
miro_makeparams(SimMotorParameters.xml )

set( SOURCES
  ${MIRO_MAKEPARAMS_SOURCES}
  SimMotor.cpp 
  SimMotorGroup.cpp
  SimMotorGroupBase.cpp
  SimWheelGroup.cpp
  SimWheelGroupSvc.cpp
)

set( HEADERS
  ${MIRO_MAKEPARAMS_HEADERS}
  ${EXPORT_FILE}
  SimMotor.h
  SimMotorGroup.h
  SimMotorGroupBase.h
  SimWheelGroup.h
  SimWheelGroupSvc.h
)

include_directories(
  ${Boost_INCLUDE_DIR}
  ${KN_INCLUDE_DIR}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${MIRO_miroXml_LIBRARY}
  ${KN_knShare_LIBRARY}
  knMotor
  knMotorImpl
  ${QT_QTXML_LIBRARY}
  ${QT_CORE_LIBRARY}
)

add_definitions(-D${EXPORT_DEFINE})

add_library(${LIB_NAME} SHARED
  ${SOURCES}
  ${HEADERS}
)

link_libraries(
  ${LIB_NAME}
)

set( TEST_TARGETS
  testMotorTrapecoidal
  testMotorTrajectories
)

foreach( TEST_TARGET ${TEST_TARGETS} )
  add_executable( ${TEST_TARGET} 
    ${TEST_TARGET}.cpp 
  )
endforeach( TEST_TARGET ${TEST_TARGETS} )

install_library(  ${LIB_NAME} )
install_targets( /bin ${TEST_TARGETS})
install_files  ( /include/${DIR_NAME} FILES ${HEADERS} )
