/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2016-2017 Dsperados <info@dsperados.com>
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>
 
 --------------------------------------------------------------------
 
 If you would like to use Grizzly for commercial or closed-source
 purposes, please contact us for a commercial license.
 
 */

#ifndef GRIZZLY_BIQUAD_COEFFICIENTS_HPP
#define GRIZZLY_BIQUAD_COEFFICIENTS_HPP

#include <cmath>
#include <stdexcept>
#include <moditone/unit/amplitude.hpp>
#include <moditone/unit/hertz.hpp>

#include <moditone/math/constants.hpp>

namespace dsp
{
    //! Biquad Coefficients
    /*! Credits to Robert Bristow-Johnson for providing the cooking formulas (see "Audio-EQ-cookbook"). 
        Notice we use "a" for feed-forward and "b" for feed-back. */
    template <class T>
    struct BiquadCoefficients
    {
        //! The a0 feed-forward coefficient (gain)
        T a0 = 0;
        
        //! The a1 feed-forward coefficient
        T a1 = 0;
        
        //! The a2 feed-forward coefficient
        T a2 = 0;
        
        //! The b1 feed-back coefficient
        T b1 = 0;
        
        //! The b2 feed-back coefficient
        T b2 = 0;
        
        //! Check filter arguments
        static void check(unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q)
        {
            // check sample rate
            if (sampleRate.value <= 0)
                throw std::invalid_argument("sampling rate <= 0");
            
            // check cut-off
            const auto nyquist = sampleRate.value / 2;
            if (cutOff.value <= 0 || cutOff.value >= nyquist)
                throw std::invalid_argument("cut-off <= 0 or >= nyquist");
            
            // check q
            if (q <= 0)
                throw std::invalid_argument("q <= 0");
        }
        
        //! Check filter arguments
        static void check(unit::hertz<float> sampleRate, unit::second<float> time, float q, float timeConstantFactor)
        {
            // check sample rate
            if (sampleRate.value <= 0)
                throw std::invalid_argument("sampling rate <= 0");
            
            // check time
            if (time.value <= 0)
                throw std::invalid_argument("time <= 0");
            
            // check q
            if (q <= 0)
                throw std::invalid_argument("q <= 0");
            
            // check time-constant-filter
            if (timeConstantFactor < 0)
                throw std::invalid_argument("time constant factor < 0");
        }
    };
    
    //! Set biquad to through pass
    template <typename T>
    void throughPass(BiquadCoefficients<T>& coefficients)
    {
        coefficients.a0 = 1;
        coefficients.a1 = 0;
        coefficients.a2 = 0;
        coefficients.b1 = 0;
        coefficients.b2 = 0;
    }
    
    //! Set biquad to no pass
    template <typename T>
    void noPass(BiquadCoefficients<T>& coefficients)
    {
        coefficients.a0 = 0;
        coefficients.a1 = 0;
        coefficients.a2 = 0;
        coefficients.b1 = 0;
        coefficients.b2 = 0;
    }
    
    //! Set biquad to low pass filtering using a cut-off frequency
    template <class T>
    void lowPass(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q)
    {
        // safety check
        BiquadCoefficients<T>::check(sampleRate, cutOff, q);
        
        const auto w = math::TWO_PI<float> * cutOff.value / sampleRate.value;
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 + alpha;
        
        coefficients.a0 = ((1 - cosw) / 2) / b0;
        coefficients.a1 = (1 - cosw) / b0;
        coefficients.a2 = ((1 - cosw) / 2) / b0;
        
        coefficients.b1 = (-2 * cosw) / b0;
        coefficients.b2 = (1 - alpha) / b0;
    }
    
    //! Set biquad to low pass filtering using a time and time constant factor
    template <class T>
    void lowPass(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::second<float> time, float q, float timeConstantFactor = 5.f)
    {
        // correct time for two-pole timed filtering
        float t = time.value *= math::SQRT_HALF<float>;
        
        // safety check
        BiquadCoefficients<T>::check(sampleRate, t, q, timeConstantFactor);
        
        const auto w = timeConstantFactor / (t * sampleRate.value);
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 + alpha;
        
        coefficients.a0 = static_cast<T>(((1 - cosw) / 2) / b0);
        coefficients.a1 = static_cast<T>((1 - cosw) / b0);
        coefficients.a2 = static_cast<T>(((1 - cosw) / 2) / b0);
        
        coefficients.b1 = static_cast<T>((-2 * cosw) / b0);
        coefficients.b2 = static_cast<T>((1 - alpha) / b0);
    }
    
    //! Set biquad to high pass filtering
    template <class T>
    void highPass(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q)
    {
        // safety check
        BiquadCoefficients<T>::check(sampleRate, cutOff, q);
        
        const auto w = math::TWO_PI<float> * cutOff.value / sampleRate.value;
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 + alpha;
        
        coefficients.a0 = ((1 + cosw) / 2) / b0;
        coefficients.a1 = (-(1 + cosw)) / b0;
        coefficients.a2 = ((1 + cosw) / 2) / b0;
        
        coefficients.b1 = (-2 * cosw) / b0;
        coefficients.b2 = (1 - alpha) / b0;
    }
    
    //! Set biquad to band pass filtering with a constant skirt gain
    template <class T>
    void bandPassConstantSkirt(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q)
    {
        // safety check
        BiquadCoefficients<T>::check(sampleRate, cutOff, q);
        
        const auto w = math::TWO_PI<float> * cutOff.value / sampleRate.value;
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 + alpha;
        
        coefficients.a0 = (q * alpha) / b0;
        coefficients.a1 = 0;
        coefficients.a2 = (-q * alpha) / b0;
        
        coefficients.b1 = (-2 * cosw) / b0;
        coefficients.b2 = (1 - alpha) / b0;
    }
    
    //! Set biquad to band pass filtering with a constant peak gain
    template <class T>
    void bandPassConstantPeak(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q)
    {
        // safety check
        BiquadCoefficients<T>::check(sampleRate, cutOff, q);
        
        const auto w = math::TWO_PI<float> * cutOff.value / sampleRate.value;
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 + alpha;
        
        coefficients.a0 = alpha / b0;
        coefficients.a1 = 0;
        coefficients.a2 = -alpha / b0;
        
        coefficients.b1 = (-2 * cosw) / b0;
        coefficients.b2 = (1 - alpha) / b0;
    }
    
    //! Set biquad to peak filtering with a constant peak gain
    template <class T>
    void peakConstantSkirt(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q, unit::decibel<float> gain)
    {
        // safety check
        BiquadCoefficients<T>::check(sampleRate, cutOff, q);
        
        const auto w = math::TWO_PI<float> * cutOff.value / sampleRate.value;
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto alpha = sinw / (2 * q);
        const auto A = std::pow(10, gain.value / 40);
        
        const auto b0 = 1 + alpha / A;
        
        coefficients.a0 = (1 + alpha * A) / b0;
        coefficients.a1 = (-2 * cosw) / b0;
        coefficients.a2 = (1 - alpha * A) / b0;
        
        coefficients.b1 = (-2 * cosw) / b0;
        coefficients.b2 = (1 - alpha / A) / b0;
    }
    
    //! Set biquad to peak filtering with a constant Q
    template <class T>
    void peakConstantQ(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q, unit::decibel<float> gain)
    {
        // safety check
        BiquadCoefficients<T>::check(sampleRate, cutOff, q);
        
        const auto w = math::TWO_PI<float> * cutOff.value / sampleRate.value;
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto alpha = sinw / (2 * q);
        const auto A = std::pow(10, gain.value / 40);
        
        const auto b0 = 1 + alpha / A;

        // Negative peak
        if (A < 1)
        {
            coefficients.a0 = (1 + alpha) / b0;
            coefficients.a1 = (-2 * cosw) / b0;
            coefficients.a2 = (1 - alpha) / b0;
            
            coefficients.b1 = (-2 * cosw) / b0;
            coefficients.b2 = (1 - alpha / A) / b0;
        }
        // Positive peak
        else
        {
            coefficients.a0 = (1 + alpha * A) / b0;
            coefficients.a1 = (-2 * cosw) / b0;
            coefficients.a2 = (1 - alpha * A) / b0;
    
            coefficients.b1 = (-2 * cosw) / b0;
            coefficients.b2 = (1 - alpha) / b0;
        }
    }
    
    //! Set biquad to low shelf filtering
    template <class T>
    void lowShelf(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q, unit::decibel<float> gain)
    {
        // safety check
        BiquadCoefficients<T>::check(sampleRate, cutOff, q);
        
        const auto w = math::TWO_PI<float> * cutOff.value / sampleRate.value;
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto A = std::pow(10, gain.value / 40);
        
        const auto beta = sqrt(A)/q;
        const auto b0 = (A + 1) + (A - 1) * cosw + beta * sinw;
        
        coefficients.a0 = (A * ((A + 1) - (A - 1) * cosw + beta * sinw)) / b0;
        coefficients.a1 = (2 * A * ((A-1) - (A + 1) * cosw)) / b0;
        coefficients.a2 = (A * ((A + 1) - (A - 1) * cosw - beta * sinw)) / b0;
        
        coefficients.b1 = (-2 * ((A - 1) + (A + 1) * cosw)) / b0;
        coefficients.b2 = ((A + 1) + (A - 1) * cosw - beta * sinw) / b0;
    }
    
    //! Set biquad to high shelf filtering
    template <class T>
    void highShelf(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q, unit::decibel<float> gain)
    {
        // safety check
        BiquadCoefficients<T>::check(sampleRate, cutOff, q);
        
        const auto w = math::TWO_PI<float> * cutOff.value / sampleRate.value;
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto A = std::pow(10, gain.value / 40);
        
        const auto beta = sqrt(A)/q;
        const auto b0 = (A + 1) - (A - 1) * cosw + beta * sinw;
        
        coefficients.a0 = (A * ((A + 1) + (A-1) * cosw + beta * sinw)) / b0;
        coefficients.a1 = (-2 * A* ((A - 1) + (A + 1) * cosw)) / b0;
        coefficients.a2 = (A * ((A + 1) + (A - 1) * cosw - beta * sinw)) / b0;
        
        coefficients.b1 = (2 *((A - 1) - (A + 1) * cosw)) / b0;
        coefficients.b2 = ((A + 1) - (A - 1) * cosw - beta * sinw) / b0;
    }
    
    //! Set biquad to notch filtering
    template <class T>
    void notch(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q)
    {
        // safety check
        BiquadCoefficients<T>::check(sampleRate, cutOff, q);
        
        const auto w = math::TWO_PI<float> * cutOff.value / sampleRate.value;
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 + alpha;
        
        coefficients.a0 = 1 / b0;
        coefficients.a1 = (-2 * cosw) / b0;
        coefficients.a2 = 1 / b0;
        coefficients.b1 = (-2 * cosw) / b0;
        coefficients.b2 = (1 - alpha) / b0;
    }
    
    //! Set biquad to all pass filtering
    template <class T>
    void allPass(BiquadCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff, float q)
    {
        // safety check
        BiquadCoefficients<T>::check(sampleRate, cutOff, q);
        
        const auto w = math::TWO_PI<float> * cutOff.value / sampleRate.value;
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 + alpha;
        
        coefficients.a0 = (1 - alpha) / b0;
        coefficients.a1 = (-2 * cosw) / b0;
        coefficients.a2 = (1 + alpha) / b0;
        coefficients.b1 = (-2 * cosw) / b0;
        coefficients.b2 = (1 - alpha) / b0;
    }
    
}

#endif
