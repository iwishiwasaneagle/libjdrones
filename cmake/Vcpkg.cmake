if(${PROJECT_NAME}_ENABLE_VCPKG)  
  #
  # If `vcpkg.cmake` (from https://github.com/microsoft/vcpkg) does not exist, download it.
  #
  if(NOT "${CMAKE_TOOLCHAIN_FILE}" MATCHES "vcpkg.cmake")
    message(
    FATAL_ERROR
        "vcpkg.cmake has not been specified within CMAKE_TOOLCHAIN_FILE"
    )
  endif()

  if(${PROJECT_NAME}_VERBOSE_OUTPUT)
		set(VCPKG_VERBOSE ON)
	endif()
    set(VCPKG_TARGET_TRIPLET x64-linux CACHE STRING "VCPKG Target Triplet to use")
endif()
