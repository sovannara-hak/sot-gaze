# Copyright 2011, Sovan Hak from Thomas Moulard (CNRS/LAAS)
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME sot-gaze)
SET(PROJECT_DESCRIPTION "FIXME")
SET(PROJECT_URL "FIXME")

SET(CUSTOM_HEADER_DIR "sot/gaze")

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.8.0")

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 1.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 1.0.0")

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS filesystem)
SEARCH_FOR_BOOST()

# Add subdirectories.
ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
