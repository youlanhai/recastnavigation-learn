file(GLOB SOURCE_FILES "*.h" "*.cpp")

add_library(Recast STATIC ${SOURCE_FILES})
