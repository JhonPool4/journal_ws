# Searches for RBDL includes and library files, including Addons.
#
# Sets the variables
#   RBDL_FOUND
#   RBDL_INCLUDE_DIR
#   RBDL_LIBRARY
#
# You can use the following components:
#   URDFReader

# Status message
MESSAGE(STATUS "Fingind RBDL and URDF . . .")

# Set variables
SET (RBDL_FOUND FALSE)
SET (RBDL_URDFReader_FOUND FALSE)

# RBDL directory path
FIND_PATH (RBDL_INCLUDE_DIR rbdl/rbdl.h
	HINTS
	~/catkin_ws/rbdl_ws/install/include
	)
# RBDL library path
FIND_LIBRARY (RBDL_LIBRARY NAMES rbdl
	PATHS
	~/catkin_ws/rbdl_ws/install/lib		
	)

# URDF Reader directory path 
FIND_PATH (RBDL_URDFReader_INCLUDE_DIR rbdl/addons/urdfreader/urdfreader.h
	HINTS
	~/catkin_ws/rbdl_ws/install/include
	)
# URDF Reader library path
FIND_LIBRARY (RBDL_URDFReader_LIBRARY NAMES rbdl_urdfreader
	PATHS
	~/catkin_ws/rbdl_ws/install/lib			
	)



IF (NOT RBDL_LIBRARY)
	MESSAGE (ERROR "Could not find RBDL")
ENDIF (NOT RBDL_LIBRARY)

IF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)
	SET (RBDL_FOUND TRUE)
ENDIF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)


IF (RBDL_URDFReader_INCLUDE_DIR AND RBDL_URDFReader_LIBRARY)
	SET (RBDL_URDFReader_FOUND TRUE)
ENDIF (RBDL_URDFReader_INCLUDE_DIR AND RBDL_URDFReader_LIBRARY)


IF (RBDL_FOUND)
   IF (NOT RBDL_FIND_QUIETLY)
      MESSAGE(STATUS "Found RBDL: ${RBDL_LIBRARY}")
   ENDIF (NOT RBDL_FIND_QUIETLY)

	 foreach ( COMPONENT ${RBDL_FIND_COMPONENTS} )
		 IF (RBDL_${COMPONENT}_FOUND)
			 IF (NOT RBDL_FIND_QUIETLY)
				 MESSAGE(STATUS "Found RBDL ${COMPONENT}: ${RBDL_${COMPONENT}_LIBRARY}")
			 ENDIF (NOT RBDL_FIND_QUIETLY)
		 ELSE (RBDL_${COMPONENT}_FOUND)
			 MESSAGE(SEND_ERROR "Could not find RBDL ${COMPONENT}")
		 ENDIF (RBDL_${COMPONENT}_FOUND)
	 endforeach ( COMPONENT )
ELSE (RBDL_FOUND)
   IF (RBDL_FIND_REQUIRED)
		 MESSAGE(SEND_ERROR "Could not find RBDL")
   ENDIF (RBDL_FIND_REQUIRED)
ENDIF (RBDL_FOUND)


IF (RBDL_URDFReader_FOUND)
	MESSAGE(STATUS "FOUND URDF_READER: ${RBDL_URDFReader_LIBRARY}")
ENDIF (RBDL_URDFReader_FOUND)

MARK_AS_ADVANCED (
	RBDL_INCLUDE_DIR
	RBDL_LIBRARY
	RBDL_URDFReader_INCLUDE_DIR
	RBDL_URDFReader_LIBRARY
	)

