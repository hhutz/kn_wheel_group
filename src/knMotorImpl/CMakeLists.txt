#
# Build knMotor library
#

set( LIB_NAME    "knMotorImpl" )
set( DIR_NAME    "knMotorImpl" )
set( EXPORT      "knMotorImpl" )
set( EXPORT_NAME "${EXPORT}_Export" )
set( EXPORT_FILE "${EXPORT_NAME}.h" )
string( TOUPPER  "${EXPORT}_BUILD_DLL" EXPORT_DEFINE )

## Miro parameters
##------------------------------------------------
#miro_makeparams(MotorParameters.xml )

set( SOURCES
  ${MIRO_MAKEPARAMS_SOURCES}
  MotorImpl.cpp 
  MotorGroupImpl.cpp
  WheelGroupImpl.cpp
  MotorGroupRepository.cpp
  MotorHelper.cpp
  MotorRepository.cpp
  WheelGroupRepository.cpp
)

set( HEADERS
  ${MIRO_MAKEPARAMS_HEADERS}
  ${EXPORT_FILE}
  MotorImpl.h
  MotorGroupImpl.h
  MotorGroupRepository.h
  MotorHelper.h 
  MotorRepository.h
  WheelGroupImpl.h
  WheelGroupRepository.h
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${KN_INCLUDE_DIR}
)


add_definitions(-D${EXPORT_DEFINE})

link_libraries(
  ${MIRO_miroXml_LIBRARY} 
  knMotor
  ${Boost_SIGNALS_LIBRARY} 
)

add_library(${LIB_NAME} SHARED
  ${SOURCES}
  ${HEADERS}
)

link_libraries(
  ${LIB_NAME}
)

set( TEST_TARGETS
)

foreach( TEST_TARGET ${TEST_TARGETS} )
  add_executable( ${TEST_TARGET} 
    ${TEST_TARGET}.cpp 
  )
endforeach( TEST_TARGET ${TEST_TARGETS} )

install_library(  ${LIB_NAME} )
##install_targets( /bin ${TEST_TARGETS})
install_files  ( /include/${DIR_NAME} FILES ${HEADERS} )
