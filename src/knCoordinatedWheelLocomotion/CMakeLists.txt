set( LIB_NAME     "knCoordinatedWheelLocomotion" )
set( DIR_NAME     "knCoordinatedWheelLocomotion" )
set( EXPORT_NAME "${LIB_NAME}_Export" )
set( EXPORT_FILE "${EXPORT_NAME}.h" )
string( TOUPPER  "${LIB_NAME}_BUILD_DLL" EXPORT_DEFINE )

# Let's double-check with Mark what the right way for this is:
##set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3" )

## Library
##------------------------------------------------
set( HEADERS
  ${EXPORT_FILE}
  CoordinatedWheelLocomotion.h
  FastestSteerAndDriveCWL.h
  FsdMatlabPlots.h
  TrapezoidProfile.h
  HalfTrapProfile.h
)

set( SOURCES
  CoordinatedWheelLocomotion.cpp
  FastestSteerAndDriveCWL.cpp
  FsdMatlabPlots.cpp
  TrapezoidProfile.cpp
  HalfTrapProfile.cpp
)

add_definitions(-D${EXPORT_DEFINE})

link_libraries (
  ${ACE_LIBRARIES}
  ${Boost_LIBRARIES}
)

include_directories(
  ${KN_INCLUDE_DIR}
  ${ACE_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${LIB_NAME} SHARED
  ${HEADERS}
  ${SOURCES}
)

## Tests
##------------------------------------------------
set( TEST_TARGETS
  test_Fsd
  test_FsdMatrix
  test_TrapezoidProfile
)

set( TEST_DATAFILES 
#  regression_baseline.txt
)

set( TEST_SCRIPTS
  test_will_pass.sh
  test_will_fail.sh
)

link_libraries( ${LIB_NAME} )

foreach( TEST_TARGET ${TEST_TARGETS} )
  add_executable( ${TEST_TARGET}
  ${TEST_TARGET}.cpp
  )
endforeach( TEST_TARGET ${TEST_TARGETS} )

##----------------------------------------
install_library( ${LIB_NAME} )
install_files  ( /include/${DIR_NAME} FILES ${HEADERS} )

install_targets( /tests      ${TEST_TARGETS})
#install_files  ( /tests/data ${TEST_DATAFILES})
install_scripts( /tests      ${TEST_SCRIPTS})
if( CTEST_ENABLED )

  add_test( test_will_pass bash ${TESTS_PATH}/test_will_pass.sh )
  
  add_test( test_will_fail bash ${TESTS_PATH}/test_will_fail.sh )
  set_tests_properties(test_will_fail PROPERTIES WILL_FAIL TRUE )
  
endif( CTEST_ENABLED )
