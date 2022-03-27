# Adapted fromm  https://github.com/ttroy50/cmake-examples/
find_program(CLANG_FORMAT "clang-format")
if(NOT CLANG_FORMAT)
  message(SEND_ERROR "clang-format not found on your \$\{PATH\}")
endif()

# Split the regex into multiple parts
set(EXCLUDE_BUILD "-path ./build -prune -o")
set(REGEX "-regextype posix-extended -regex")
set(CPP_EXT "'.*\\.(cpp|cxx|cc|hpp|hxx|h)'")
set(TRIM_OUT "-print |  tr '\\n' ';'")

# Get all project files
execute_process(
  COMMAND bash -c "find . ${EXCLUDE_BUILD} ${REGEX} ${CPP_EXT} ${TRIM_OUT}"
  OUTPUT_VARIABLE ALL_SOURCES
  OUTPUT_STRIP_TRAILING_WHITESPACE
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_custom_target(clang-format ALL
                  COMMENT "Checking clang-format changes"
                  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                  COMMAND ${CLANG_FORMAT} -Werror
                          --dry-run --ferror-limit=1 -style=file ${ALL_SOURCES})
