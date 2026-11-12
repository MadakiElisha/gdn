# Writes OUT (gdn_version.hpp) with `git describe` of the source tree (R1.11: every run log
# names the build it came from). Runs at every build; rewrites the file only when the
# version changes, so nothing recompiles otherwise. "-dirty": uncommitted changes.
execute_process(COMMAND git describe --always --dirty --abbrev=10
                WORKING_DIRECTORY "${SRC}" OUTPUT_VARIABLE V
                OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE rc)
if(NOT rc EQUAL 0 OR "${V}" STREQUAL "")
  set(V "unknown")
endif()
set(CONTENT "#pragma once\n#define GDN_VERSION \"${V}\"\n")
set(OLD "")
if(EXISTS "${OUT}")
  file(READ "${OUT}" OLD)
endif()
if(NOT "${OLD}" STREQUAL "${CONTENT}")
  file(WRITE "${OUT}" "${CONTENT}")
endif()
