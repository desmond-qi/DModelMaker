# - Config file for the DMujoMk package
# It defines the following variables
#  DMujoMk_INCLUDE_DIR - include directories for DMujoMk
#  DMujoMk_SRC_DIR     - src     directories for DMujoMk
#  DMujoMk_LIB_DIR     - lib     directories for DMujoMk
 
# Compute paths
if(1)
    set(DMujoMk_INCLUDE_DIR "D:/_pkg/DPackages/DMujoMk/include")
endif()
if(0)
    set(DMujoMk_LIB_DIR     "D:/_pkg/DPackages/DMujoMk/lib"    )
endif()
if(0)
    set(DMujoMk_SRC_DIR     "D:/_pkg/DPackages/DMujoMk/src"    )
endif()

# set paths
message("-- [DMujoMk]: Package found!")
if(DEFINED DMujoMk_INCLUDE_DIR)
    include_directories(${DMujoMk_INCLUDE_DIR})
    message("-- [DMujoMk]: Include ${DMujoMk_INCLUDE_DIR}")
endif()
if(DEFINED DMujoMk_LIB_DIR)
    link_directories(${DMujoMk_LIB_DIR})
    message("-- [DMujoMk]: Link ${DMujoMk_LIB_DIR}")
endif()
if(DEFINED DMujoMk_SRC_DIR)
    aux_source_directory(${DMujoMk_SRC_DIR} DMujoMk_FILES)
    message("-- [DMujoMk]: Can use DMujoMk_SRC_FILES")
endif()
