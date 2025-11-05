add_library(algorithm)

target_sources(algorithm PRIVATE
    ~/Desktop/lib/algorithm/fltr.c
)

target_include_directories(algorithm PUBLIC
    ~/Desktop/lib/algorithm
)
