message(STATUS "[point_labeler] Fetching glow.")

include(FetchContent)
FetchContent_Declare(glow GIT_REPOSITORY https://github.com/jbehley/glow.git)
                    
FetchContent_MakeAvailable(glow)

# TODO: Can we get rid of this explicit inclusion?
include(GenCppFile)
include(GlowShaderCompilation)
