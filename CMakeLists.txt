# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ROPODS_DRIVERS)
  add_subdirectory(drivers)
endif()

if(CONFIG_ROPODS_LIBRARIES)
  add_subdirectory(lib)
endif()
