file(GLOB SOURCE_FILES "*.h" "*.cpp")

add_library(Detour STATIC ${SOURCE_FILES})
