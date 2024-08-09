if(${PROJECT_NAME_UPPERCASE}_ENABLE_DOXYGEN)
    set(DOXYGEN_CALLER_GRAPH YES)
    set(DOXYGEN_CALL_GRAPH YES)
    set(DOXYGEN_EXTRACT_ALL YES)
    set(DOXYGEN_OUTPUT_DIRECTORY docs)

    find_package(Doxygen REQUIRED dot)
    doxygen_add_docs(doxygen-docs ${sources} ${headers})

    verbose_message("Doxygen has been setup and documentation is now available.")
endif()
