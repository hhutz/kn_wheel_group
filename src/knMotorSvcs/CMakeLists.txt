set( LIB_NAME     "knMotorSvcs" )
set( DIR_NAME     "knMotorSvcs" )
set( EXPORT_NAME "${DIR_NAME}_Export" )
set( EXPORT_FILE "${EXPORT_NAME}.h" )
string( TOUPPER  "${DIR_NAME}_BUILD_DLL" EXPORT_DEFINE )

## Miro parameters
##------------------------------------------------
miro_makeparams( MotorSvcsParameters.xml )

## Library
##------------------------------------------------
set( HEADERS
  ${MIRO_MAKEPARAMS_HEADERS}
  ${EXPORT_FILE}
  VelocityController.h
  WheelGroupJointProvider.h
  WheelGroupSvc.h
)

set( SOURCES
  ${MIRO_MAKEPARAMS_SOURCES}
  VelocityController.cpp
  WheelGroupJointProvider.cpp
  WheelGroupSvc.cpp
)

include_directories(SYSTEM  ${RTIDDS_INCLUDE_DIR})

include_directories (
  ${KNRAPID_INCLUDE_DIR}
  ${KNHW_INCLUDE_DIR}
  ${GRI_INCLUDE_DIR}
)

link_libraries (
  knMotor
  knMotorImpl
  knCoordinatedWheelLocomotion
  ${RTIDDS_LIBRARIES}
  ${KNDDS_knDds_LIBRARY}
  ${KNRAPID_rapidDds_LIBRARY}
  ${KNRAPID_rapidExtArcDds_LIBRARY}
  ${KNRAPID_rapidUtil_LIBRARY}
  ${KNRAPID_rapidIo_LIBRARY}
  ${Boost_SIGNALS_LIBRARY} 
)

add_library(${LIB_NAME} SHARED
  ${HEADERS}
  ${SOURCES}
  ${EXPORT_FILE}
)

# export define only applies to lib; must go *after* add_library() call
add_target_definitions( ${LIB_NAME} ${EXPORT_DEFINE} )
# DDS defines apply to lib and test targets (all projects in this directory)
add_srcdir_definitions( ${RTIDDS_DEFINES} )

##----------------------------------------
install_library( ${LIB_NAME} )
install_files  ( /include/${DIR_NAME} FILES ${HEADERS} )
