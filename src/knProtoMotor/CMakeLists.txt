#
# Build knProtoMotor library
#

set( LIB_NAME    "knProtoMotor" )
set( DIR_NAME    "knProtoMotor" )
set( EXPORT      "knProtoMotor" )
set( EXPORT_NAME "${EXPORT}_Export" )
set( EXPORT_FILE "${EXPORT_NAME}.h" )
string( TOUPPER  "${EXPORT}_BUILD_DLL" EXPORT_DEFINE )

## Miro parameters
##------------------------------------------------
miro_makeparams( ProtoMotorParameters.xml )

## Library
##------------------------------------------------
set( SOURCES
  ${MIRO_MAKEPARAMS_SOURCES}
  WheelGroup.cpp
  MotorInterface.cpp
)

set( HEADERS
  ${MIRO_MAKEPARAMS_HEADERS}
  ${EXPORT_FILE}
  WheelGroup.h
  MotorInterface.h
)

include_directories(
  ${KN_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

add_definitions(-D${EXPORT_DEFINE})

link_libraries(
  ${catkin_LIBRARIES}
  ${MIRO_miroCore_LIBRARY}
  ${KN_knShare_LIBRARY}
  knMotor
  knMotorImpl
  ${QT_QTXML_LIBRARY}
  ${QT_CORE_LIBRARY}
)

# add platform thread libs
link_libraries( ${CMAKE_THREAD_LIBS_INIT} )

add_library(${LIB_NAME} SHARED
  ${SOURCES}
  ${HEADERS}
)

set( TEST_TARGETS
  testMotorInterface
  motorServer
)

foreach( TEST_TARGET ${TEST_TARGETS} )
  add_executable( ${TEST_TARGET} 
    ${TEST_TARGET}.cpp 
  )
  target_link_libraries(${TEST_TARGET}
    ${LIB_NAME}
    ${qt_LIBRARIES}
  )
endforeach( TEST_TARGET ${TEST_TARGETS} )

install_library( ${LIB_NAME} )
install_files  ( /include/${DIR_NAME} FILES ${HEADERS} )
install_targets( /bin ${TEST_TARGETS} )
