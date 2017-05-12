cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_OSX_DEPLOYMENT_TARGET "10.9" CACHE STRING "Minimum OS X deployment version")

# Create the project
project(Grizzly CXX)

# Add global definitions and include directories
add_definitions(-std=c++14 -Wall -Werror)
include_directories(/usr/local/include)

# Setup the common headers and sources
set(HEADERS all_pass_filter.hpp analog_one_pole_filter.hpp analytic_transform.hpp biquad.hpp biquad_coefficients.hpp cascade.hpp cepstrum.hpp circular_buffer.hpp comb_filter.hpp convolution.hpp crossover_filter.hpp delay.hpp dynamic.hpp envelope_follower.hpp fast_fourier_transform.hpp fast_fourier_transform_base.hpp first_order_filter_coefficients.hpp first_order_filter.hpp gordon_smith_oscillator.hpp hilbert_transform.hpp high_frequency_content.hpp impulse_response.hpp ladder_filter.hpp mid_side.hpp multi_crossover_filter.hpp multi_tap_resonator.hpp phase_generator.hpp pink_noise.hpp poly_blep.hpp saw.hpp segment_envelope.hpp short_time_fourier_transform.hpp sine.hpp spectral_centroid.hpp spectrum.hpp square.hpp state_variable_filter.hpp triangle.hpp waveform.hpp window.hpp yin.hpp z_transform.hpp)
set(SOURCES cepstrum.cpp fast_fourier_transform_base.cpp)
source_group(\\ FILES ${HEADERS} ${SOURCES})
install (FILES ${HEADERS} DESTINATION include/grizzly)

# Ooura
set(OOURA_HEADERS ooura/fast_fourier_transform_ooura.hpp ooura/fftsg.hpp)
set(OOURA_SOURCES ooura/fast_fourier_transform_ooura.cpp ooura/fftsg.cpp)
source_group(\\Ooura FILES ${OOURA_HEADERS} ${OOURA_SOURCES})
install (FILES ${OOURA_HEADERS} DESTINATION include/grizzly/ooura)

list(APPEND HEADERS ${OOURA_HEADERS})
list(APPEND SOURCES ${OOURA_SOURCES})

# Apple
if (APPLE)
    set(APPLE_HEADERS apple/fast_fourier_transform_accelerate.hpp)
	set(APPLE_SOURCES apple/fast_fourier_transform_accelerate.cpp)
    source_group(\\Apple FILES ${APPLE_HEADERS} ${APPLE_SOURCES})
    install (FILES ${APPLE_HEADERS} DESTINATION include/grizzly/apple)

    list(APPEND HEADERS ${APPLE_HEADERS})
        list(APPEND SOURCES ${APPLE_SOURCES})
endif()

# Create the target
add_library(Grizzly ${HEADERS} ${SOURCES})
install(TARGETS Grizzly DESTINATION lib)

# Handle different cases if this will be a static or shared lib
if (BUILD_SHARED_LIBS)
    set_target_properties(Grizzly PROPERTIES OUTPUT_NAME grizzly)

    if (APPLE)
        find_library(Accelerate Accelerate REQUIRED)
        target_link_libraries(Grizzly ${Accelerate})
    endif(APPLE)
else (BUILD_SHARED_LIBS)
    set_target_properties(Grizzly PROPERTIES OUTPUT_NAME grizzly_static)
endif (BUILD_SHARED_LIBS)
