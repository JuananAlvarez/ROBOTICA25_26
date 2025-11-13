# Sources set
LIST(APPEND SOURCES
        ../src/specificworker.cpp
        ../src/hungarian.cpp
        ../src/room_detector.cpp
        ../src/ransac_line_detector.cpp
        ../src/door_detector.cpp          # <---
        ../src/qcustomplot.cpp            # <---
        ../src/time_series_plotter.cpp    # <---
)

# Headers set
LIST(APPEND HEADERS
        ../src/specificworker.h
        ../src/common_types.h
        ../src/hungarian.h
        ../src/munkres.hpp
        ../src/room_detector.h
        ../src/ransac_line_detector.h
        ../src/door_detector.h            # <---
        ../src/image_processor.h          # <---
        ../src/nominal_room.h
        ../src/qcustomplot.h              # <---
        ../src/time_series_plotter.h      # <---

)

INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
SET(LIBS ${LIBS} tbb Qt6PrintSupport)

LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)


