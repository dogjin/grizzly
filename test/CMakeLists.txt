cmake_minimum_required(VERSION 3.5.1)

project(grizzly-test)

add_definitions(-std=c++14 -Wall -Werror)
remove_definitions(-Wconversion)
include_directories(/usr/local/include)

set(SOURCES
    main.cpp
    all_pass_filter.cpp
    analog_one_pole_filter.cpp
    analytic_transform.cpp
    biquad.cpp
    cepstrum.cpp
    circular_buffer.cpp
    comb_filter.cpp
    complex.cpp
    convolution.cpp
    delay.cpp
    # down_sample.cpp
    # dynamic.cpp
    even_odd.cpp
    fast_fourier_transform_ooura.cpp
    first_order_filter.cpp
    gordon_smith_oscillator.cpp
    hilbert_transform.cpp
    high_frequency_content.cpp
    impulse_response.cpp
    mid_side.cpp
    multi_tap_resonator.cpp
    segment_envelope.cpp
    spectral_centroid.cpp
    # up_sample.cpp
    waveform.cpp
    window.cpp
    z_transform.cpp)

add_executable(grizzly-test ${SOURCES})
target_sources(grizzly-test PRIVATE ${SOURCES})
source_group(\\ FILES ${SOURCES})

target_link_libraries(grizzly-test Grizzly)

if (APPLE)
    target_sources(grizzly-test PRIVATE fast_fourier_transform_accelerate.cpp)
    source_group(\\ FILES fast_fourier_transform_accelerate.cpp)
	find_library(Accelerate Accelerate REQUIRED)
	target_link_libraries(grizzly-test ${Accelerate})
endif (APPLE)
