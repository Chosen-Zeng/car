add_library(lib)

target_sources(lib PRIVATE
    ~/Desktop/lib/algorithm/fltr.c
)

target_include_directories(lib PUBLIC
    ~/Desktop/lib/STM32
    ~/Desktop/lib/algorithm
)