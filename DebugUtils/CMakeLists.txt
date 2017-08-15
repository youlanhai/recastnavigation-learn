file(GLOB SOURCE_FILES "*.h" "*.cpp")

include_directories(../Detour ../DetourTileCache ../Recast)
add_library(DebugUtils STATIC ${SOURCE_FILES})
