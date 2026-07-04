# Host unit tests: one executable per test_*.c file, auto-discovered so
# module implementation agents never need to edit this file — drop a
# tests/test_<module>.c and reconfigure.

add_compile_definitions(HOST_TEST_BUILD)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g -O0")

file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test_*.c)

foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)

    # A test may need one or more SUFST sources alongside it, named either
    # test_<module>.deps or test_<module>.c.deps (one relative path per
    # line, relative to SUFST/Src) - both spellings are accepted since it's
    # an easy naming mismatch to make.
    set(extra_sources "")
    foreach(test_deps_file ${test_name}.deps ${test_name}.c.deps)
        set(test_deps_file ${CMAKE_CURRENT_SOURCE_DIR}/${test_deps_file})
        if(EXISTS ${test_deps_file})
            file(STRINGS ${test_deps_file} dep_lines)
            foreach(dep_line ${dep_lines})
                if(dep_line)
                    list(APPEND extra_sources ${CMAKE_SOURCE_DIR}/SUFST/Src/${dep_line})
                endif()
            endforeach()
        endif()
    endforeach()
    list(REMOVE_DUPLICATES extra_sources)

    add_executable(${test_name} ${test_src} ${extra_sources})
    target_include_directories(${test_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/SUFST/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(${test_name} PRIVATE m)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
