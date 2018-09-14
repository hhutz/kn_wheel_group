#
# Build knMotor library
#

set( LIB_NAME    "knMotor" )
set( DIR_NAME    "knMotor" )
set( EXPORT      "knMotor" )
set( EXPORT_NAME "${EXPORT}_Export" )
set( EXPORT_FILE "${EXPORT_NAME}.h" )
string( TOUPPER  "${EXPORT}_BUILD_DLL" EXPORT_DEFINE )

## Miro parameters
##------------------------------------------------
miro_makeparams(MotorParameters.xml )

set( SOURCES
  ${MIRO_MAKEPARAMS_SOURCES}
  HwReactor.cpp
  Motor.cpp
  MotorBase.cpp 
  MotorFuture.cpp
  MotorGroup.cpp
  MotorGroupSample.cpp
  MotorProfile.cpp
  MotorSample.cpp
  MotorState.cpp
  WheelGroup.cpp
  WheelGroupSample.cpp
)

set( HEADERS
  ${MIRO_MAKEPARAMS_HEADERS}
  ${EXPORT_FILE}
  HwReactor.h
  Motor.h
  MotorBase.h
  MotorFuture.h
  MotorGroup.h
  MotorGroupSample.h
  MotorProfile.h
  MotorSample.h
  MotorState.h
  WheelGroup.h
  WheelGroupSample.h
)

add_definitions(-D${EXPORT_DEFINE})

include_directories( 
  ${Boost_INCLUDE_DIR}
  ${KN_INCLUDE_DIR}
)


link_libraries(
  ${MIRO_miroXml_LIBRARY} 
  ${KN_knShare_LIBRARY}
  ${KN_knMotorShare_LIBRARY}
  ${Boost_SIGNALS_LIBRARY} 
  ${catkin_LIBRARIES}
)

add_library(${LIB_NAME} SHARED
  ${SOURCES}
  ${HEADERS}
)

link_libraries(
  ${LIB_NAME}
)

set( TEST_TARGETS
  testMotorErrors
)

foreach( TEST_TARGET ${TEST_TARGETS} )
  add_executable( ${TEST_TARGET} 
    ${TEST_TARGET}.cpp 
  )
endforeach( TEST_TARGET ${TEST_TARGETS} )

install_library(  ${LIB_NAME} )
install_files  ( /include/${DIR_NAME} FILES ${HEADERS} )
install_targets( /bin ${TEST_TARGETS} )
