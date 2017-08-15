file(GLOB SOURCE_FILES "*.h" "*.cpp")

include_directories(../Detour)
add_library(DetourTileCache STATIC ${SOURCE_FILES})
