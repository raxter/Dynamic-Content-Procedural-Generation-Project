MESSAGE( FATAL_ERROR "Find module not implemented" )

# Locate OpenTissue
# This module defines
# This module defines XXX_FOUND, XXX_INCLUDE_DIRS and XXX_LIBRARIES standard variables

#FIND_PACKAGE(Boost

#SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREAD ON)
SET(Boost_ADDITIONAL_VERSIONS "1.37.0")
#FIND_PACKAGE(Boost 1.37.0 COMPONENTS numeric )
FIND_PACKAGE(Boost 1.37.0)


SET(OPENTISSUE_HEADERS Cooking Foundation Character Physics PhysXLoader)		# Extensions
SET(OPENTISSUE_INCLUDE_DIRS)


FOREACH(CUR_DIR ${OPENTISSUE_HEADERS})
	FIND_PATH(OPENTISSUE_${CUR_DIR}_INCLUDE_DIR
		NAMES "${CUR_DIR}.h" "Nx${CUR_DIR}.h"
		HINTS
		$ENV{OPENTISSUE_DIR}
		$ENV{OPENTISSUE_PATH}
		${ADDITIONAL_SEARCH_PATHS}
		PATH_SUFFIXES include "SDKs/${CUR_DIR}/include"
		PATHS
			~/Library/Frameworks
			/Library/Frameworks
			/usr/local
			/usr
			/sw # Fink
			/opt/local # DarwinPorts
			/opt/csw # Blastwave
			/opt
	)

	# Combine all dirs to one variable
	IF(OPENTISSUE_${CUR_DIR}_INCLUDE_DIR AND NOT OPENTISSUE_INCLUDE_DIR_ERROR)
		LIST(APPEND OPENTISSUE_INCLUDE_DIRS "${OPENTISSUE_${CUR_DIR}_INCLUDE_DIR}")
	ENDIF()
ENDFOREACH()



FIND_LIBRARY(OPENTISSUE_LIBRARY
	NAMES box2d
	HINTS
	$ENV{OPENTISSUE_DIR}
	$ENV{OPENTISSUE_PATH}
	${ADDITIONAL_SEARCH_PATHS}
	PATH_SUFFIXES lib64 lib lib/release Library
	PATHS
		~/Library/Frameworks
		/Library/Frameworks
		/usr/local
		/usr
		/sw
		/opt/local
		/opt/csw
		/opt
)

FIND_LIBRARY(OPENTISSUE_LIBRARY_DEBUG 
	NAMES box2dd box2d_d
	HINTS
	$ENV{OPENTISSUE_DIR}
	$ENV{OPENTISSUE_PATH}
	${ADDITIONAL_SEARCH_PATHS}
	PATH_SUFFIXES lib64 lib lib/debug Library
	PATHS
		~/Library/Frameworks
		/Library/Frameworks
		/usr/local
		/usr
		/sw
		/opt/local
		/opt/csw
		/opt
)


# handle the QUIETLY and REQUIRED arguments and set CURL_FOUND to TRUE if 
# all listed variables are TRUE
SET(OPENTISSUE_INCLUDE_DIR_FULL_LIST)
FOREACH(CUR_DIR ${OPENTISSUE_HEADERS})
	LIST(APPEND OPENTISSUE_INCLUDE_DIR_FULL_LIST "OPENTISSUE_${CUR_DIR}_INCLUDE_DIR")
ENDFOREACH()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OPENTISSUE DEFAULT_MSG OPENTISSUE_LIBRARY ${OPENTISSUE_INCLUDE_DIR_FULL_LIST})

IF(OPENTISSUE_FOUND)
	FIND_PACKAGE_SET_TARGET_LIBRARIES(OPENTISSUE)
	# OPENTISSUE_INCLUDE_DIRS has been set before
ELSE()
	SET(OPENTISSUE_LIBRARIES)
	SET(OPENTISSUE_INCLUDE_DIRS)
ENDIF()
