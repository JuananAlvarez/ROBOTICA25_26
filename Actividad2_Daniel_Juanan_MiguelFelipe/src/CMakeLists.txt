# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
        ../src/hungarian.cpp
        ../src/room_detector.cpp
        ../src/ransac_line_detector.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h

)

INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)

LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)

