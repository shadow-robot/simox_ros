# - Find SoQt (Open Inventor Qt Integration)
# SoQt is an implementation of the Open Inventor API.
# It provides data structures and algorithms for 3D visualization
# http://www.coin3d.org/
#
# This module defines the following variables
#  SoQt_FOUND         - system has SoQt - Open Inventor Qt Integration
#  SoQt_INCLUDE_DIRS  - where the SoQt include directory can be found
#  SoQt_LIBRARIES     - Link to this to use SoQt
#
 

IF (WIN32)
  IF (CYGWIN)

    FIND_PATH(SoQt_INCLUDE_DIRS Inventor/Qt/SoQt.h)
    FIND_LIBRARY(SoQt_LIBRARIES SoQt)

  ELSE (CYGWIN)
    FIND_PATH(SoQt_INCLUDE_DIRS Inventor/Qt/SoQt.h "$ENV{COINDIR}/include" "$ENV{Coin3D_DIR}/include")
    FIND_LIBRARY(SoQt_LIBRARY_RELEASE soqt1 "$ENV{COINDIR}/lib" "$ENV{Coin3D_DIR}/lib")
    FIND_LIBRARY(SoQt_LIBRARY_DEBUG soqt1d "$ENV{COINDIR}/lib" "$ENV{Coin3D_DIR}/lib")

    IF (SoQt_LIBRARY_DEBUG AND SoQt_LIBRARY_RELEASE)
      SET(SoQt_LIBRARIES optimized ${SoQt_LIBRARY_RELEASE}
                           debug ${SoQt_LIBRARY_DEBUG})
    ELSE (SoQt_LIBRARY_DEBUG AND SoQt_LIBRARY_RELEASE)
      IF (SoQt_LIBRARY_DEBUG)
        SET (SoQt_LIBRARIES ${SoQt_LIBRARY_DEBUG})
      ENDIF (SoQt_LIBRARY_DEBUG)
      IF (SoQt_LIBRARY_RELEASE)
        SET (SoQt_LIBRARIES ${SoQt_LIBRARY_RELEASE})
      ENDIF (SoQt_LIBRARY_RELEASE)
    ENDIF (SoQt_LIBRARY_DEBUG AND SoQt_LIBRARY_RELEASE)

  ENDIF (CYGWIN)

ELSE (WIN32)
  IF(APPLE)
    FIND_PATH(SoQt_INCLUDE_DIRS Inventor/Qt/SoQt.h
     /Library/Frameworks/Inventor.framework/Headers 
    )
    FIND_LIBRARY(SoQt_LIBRARIES SoQt
      /Library/Frameworks/Inventor.framework/Libraries
    )   
    SET(SoQt_LIBRARIES "-framework SoQt" CACHE STRING "SoQt library for OSX")
  ELSE(APPLE)

    FIND_PATH(SoQt_INCLUDE_DIRS NAMES Inventor/Qt/SoQt.h PATHS "$ENV{SoQt_DIR}/include" "$ENV{COIN3D_DIR}/include" "$ENV{Coin3D_DIR}/include" /usr/include /usr/local/include NO_DEFAULT_PATH)
    FIND_LIBRARY(SoQt_LIBRARIES NAMES SoQt PATHS "$ENV{SoQt_DIR}/lib" "$ENV{COIN3D_DIR}/lib" "$ENV{Coin3D_DIR}/lib" /usr/lib /usr/local/lib NO_DEFAULT_PATH)

  ENDIF(APPLE)

ENDIF (WIN32)

# handle the QUIETLY and REQUIRED arguments and set SoQt_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(SoQt DEFAULT_MSG SoQt_LIBRARIES SoQt_INCLUDE_DIRS)

MARK_AS_ADVANCED(SoQt_INCLUDE_DIRS SoQt_LIBRARIES )


