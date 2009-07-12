# Install script for directory: /home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(libpal)$")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpal.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpal.so"
         RPATH "")
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpal.so")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/lib/libpal.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpal.so")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpal.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpal.so")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(libpal)$")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(libpal-dev)$")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pal/pal" TYPE FILE FILES
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/Config.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/ConfigStatic.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/ConfigVersion.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/pal.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palActuators.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palBase.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palBodies.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palBodyBase.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palCollision.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palExtraActuators.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palFactory.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palFluid.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palGeometry.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palLinks.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palMaterials.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palMath.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palSensors.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palSettings.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palSolver.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palStatic.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palTerrain.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal/palVehicle.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(libpal-dev)$")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(libpal-dev)$")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pal/framework" TYPE FILE FILES
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/framework/common.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/framework/empty.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/framework/errorlog.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/framework/factory.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/framework/factoryconfig.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/framework/managedmemoryobject.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/framework/os.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/framework/osfs.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/framework/statuscode.h"
    "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/framework/statusobject.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(libpal-dev)$")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(libpal-dev)$")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pal/pal_i" TYPE FILE FILES "/home/richard/workspace/cpp/Dynamic-Content-Procedural-Generation-Project/pal/pal_i/hull.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(libpal-dev)$")

