cmake_minimum_required(VERSION 3.5.1)

if (APPLE)
	# NEEDS TO BE DONE BEFORE project()
	set(CMAKE_OSX_DEPLOYMENT_TARGET "10.9" CACHE STRING "Minimum OS X deployment version")
endif (APPLE)

# Create the project
project(Grizzly CXX)

# Setup the common headers and sources
set(HEADERS all_pass_filter.hpp analog_envelope.hpp analog_one_pole_filter.hpp analytic_transform.hpp bicube.hpp bicube_coefficients.hpp biquad.hpp biquad_coefficients.hpp cascade.hpp cepstrum.hpp circular_buffer.hpp comb_filter.hpp complex.hpp convolution.hpp crossover_filter.hpp delay.hpp differentiator.hpp diode_ladder_filter.hpp dynamic.hpp envelope_follower.hpp even_odd.hpp fast_fourier_transform.hpp fast_fourier_transform_base.hpp first_order_filter_coefficients.hpp first_order_filter.hpp frequency_estimation.hpp generator.hpp gordon_smith_oscillator.hpp hard_clip.hpp hilbert_transform.hpp high_frequency_content.hpp impulse_response.hpp ladder_filter.hpp mid_side.hpp multi_band_filter.hpp multi_crossover_filter.hpp multi_tap_resonator.hpp newton_solver.hpp normalized_saturator.hpp pan.hpp parameter_smoother.hpp phasor.hpp pink_noise.hpp poly_blamp.hpp poly_blep.hpp rounded_clip.hpp saw.hpp segment_envelope.hpp short_time_fourier_transform.hpp sine.hpp slew_limiter.hpp sliding_difference.hpp sliding_dot_product.hpp spectral_centroid.hpp square.hpp state_variable_filter.hpp step_sequencer.hpp topology_preserving_one_pole_filter.hpp triangle.hpp waveform.hpp window.hpp z_transform.hpp)

set(SOURCES fast_fourier_transform_base.cpp generator.cpp phasor.cpp)

source_group(\\ FILES ${HEADERS} ${SOURCES})

# Ooura
set(OOURA_HEADERS ooura/fast_fourier_transform_ooura.hpp ooura/fftsg.hpp)
set(OOURA_SOURCES ooura/fast_fourier_transform_ooura.cpp ooura/fftsg.cpp)
source_group(\\Ooura FILES ${OOURA_HEADERS} ${OOURA_SOURCES})

list(APPEND HEADERS ${OOURA_HEADERS})
list(APPEND SOURCES ${OOURA_SOURCES})

# Apple
if (APPLE)
	add_definitions(-std=c++14 -Wall -Werror -Wconversion)
	include_directories(/usr/local/include)

	install (FILES ${HEADERS} DESTINATION include/moditone/grizzly)
    install (FILES ${OOURA_HEADERS} DESTINATION include/moditone/grizzly/ooura)

    set(APPLE_HEADERS apple/fast_fourier_transform_accelerate.hpp)
	set(APPLE_SOURCES apple/fast_fourier_transform_accelerate.cpp)
    source_group(\\Apple FILES ${APPLE_HEADERS} ${APPLE_SOURCES})
    install (FILES ${APPLE_HEADERS} DESTINATION include/moditone/grizzly/apple)

    list(APPEND HEADERS ${APPLE_HEADERS})
    list(APPEND SOURCES ${APPLE_SOURCES})

    add_subdirectory(test)
endif()

# Create the target
add_library(Grizzly ${HEADERS} ${SOURCES})
install(TARGETS Grizzly DESTINATION lib)

if(WIN32)

	add_definitions(/std:latest /Wall /WX-)
	include_directories("C:/Program Files")

	install (FILES ${HEADERS} DESTINATION moditone/grizzly)
	install (FILES ${OOURA_HEADERS} DESTINATION moditone/grizzly/Ooura)

endif(WIN32)

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
