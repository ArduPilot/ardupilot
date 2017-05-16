if(EXISTS "${PROJECT_SOURCE_DIR}/.gitmodules")
  message(STATUS "Updating submodules")
  message(STATUS "(this can take a while, please be patient)")

  ### set the direcory where the submodules live
  set(GIT_SUBMODULES_DIRECTORY modules)

  ### First, get all submodules in
  if(${GIT_SUBMODULES_CHECKOUT_QUIET})
      execute_process(
	  COMMAND             git submodule update --init --recursive
	  WORKING_DIRECTORY   ${PROJECT_SOURCE_DIR}
	  OUTPUT_QUIET
	  ERROR_QUIET
      )
  else()
      execute_process(
	  COMMAND             git submodule update --init --recursive
	  WORKING_DIRECTORY   ${PROJECT_SOURCE_DIR}
      )
  endif()
endif() 
