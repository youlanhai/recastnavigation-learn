file(GLOB SOURCE_FILES "*.h" "*.cpp")

include_directories(../Detour ../Recast)
add_library(DetourCrowd STATIC ${SOURCE_FILES})
