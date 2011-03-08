# Copyright 2011, Thomas Moulard (CNRS/LAAS)
#

INCLUDE(../cmake/python.cmake)

FUNCTION(COMPILE_PLUGIN NAME SOURCES ENTITIES)
  ADD_LIBRARY(${NAME} SHARED ${SOURCES})
  SET_TARGET_PROPERTIES(${lib} PROPERTIES
    PREFIX ""
    SOVERSION ${PROJECT_VERSION})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-mal)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} sot-core)

  INSTALL(TARGETS ${NAME} DESTINATION lib/plugin)

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${NAME})
  ADD_DEPENDENCIES(${NAME} MKDIR_${PYTHON_LIBRARY_NAME})
  ADD_CUSTOM_TARGET(MKDIR_${PYTHON_LIBRARY_NAME}
    mkdir -p dynamic_graph/sot/dynamics/${PYTHON_LIBRARY_NAME}
    )
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/gaze/${PYTHON_LIBRARY_NAME}"
    ${NAME}
    sot/gaze/${PYTHON_LIBRARY_NAME}/wrap
    )
ENDFUNCTION()


COMPILE_PLUGIN(projection2D sotProjection2D.cpp Projection2D)


EXEC_PROGRAM(
  ${PYTHON_EXECUTABLE}
  ARGS
  "-c \"from distutils import sysconfig; print sysconfig.get_python_lib(0,0,prefix='')\""
  OUTPUT_VARIABLE PYTHON_SITELIB
  )

INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/gaze/__init__.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/gaze/sot-gaze-tests.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/gaze
)
