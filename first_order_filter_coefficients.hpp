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

#ifndef GRIZZLY_FIRST_ORDER_COEFFICIENTS_HPP
#define GRIZZLY_FIRST_ORDER_COEFFICIENTS_HPP

#include <cmath>
#include <stdexcept>
#include <unit/hertz.hpp>
#include <unit/time.hpp>

#include <dsperados/math/constants.hpp>
#include <dsperados/math/utility.hpp>

namespace dsp
{
    //! Coefficients for a first-order, one-pole/one-zero filter
    template <class T>
    struct FirstOrderCoefficients
    {
        //! The a0 feed-forward coefficient (gain)
        T a0 = 0;
        
        //! The a1 feed-forward coefficient
        T a1 = 0;
        
        //! The b1 feed-back coefficient
        T b1 = 0;
        
        //! Check if sample rate is valid
        static void checkSampleRate(unit::hertz<float> sampleRate)
        {
            if (sampleRate.value <= 0)
                throw std::invalid_argument("sampling rate <= 0");
        }
        
        //! Check arguments for frequency filters
        static void check(unit::hertz<float> sampleRate, unit::hertz<float> cutOff)
        {
            // check sample rate
            checkSampleRate(sampleRate);
            
            // check cut-off
            const auto nyquist = sampleRate.value / 2;
            if (cutOff.value <= 0 || cutOff.value >= nyquist)
                throw std::invalid_argument("cut-off <= 0 or >= nyquist");
        }
        
        //! Check arguments for timed filters
        static void check(unit::hertz<float> sampleRate, unit::second<float> time, float timeConstantFactor)
        {
            // check sample rate
            checkSampleRate(sampleRate);
            
            // check time
            if (time.value <= 0)
                throw std::invalid_argument("time <= 0");
            
            // check time-constant-filter
            if (timeConstantFactor < 0)
                throw std::invalid_argument("time constant factor < 0");
        }
    };
    
    //! Set filter to through pass
    template <typename T>
    void throughPass(FirstOrderCoefficients<T>& coefficients)
    {
        coefficients.a0 = 1;
        coefficients.a1 = 0;
        coefficients.b1 = 0;
    }
    
    //! Set filter to no pass
    template <typename T>
    void noPass(FirstOrderCoefficients<T>& coefficients)
    {
        coefficients.a0 = 0;
        coefficients.a1 = 0;
        coefficients.b1 = 0;
    }
    
    //! Set filter to low pass using one zero, given a default gain coefficient of 0.5 (coefficient is clamped in range 0 to 1)
    template <typename T>
    void lowPassOneZero(FirstOrderCoefficients<T>& coefficients, unit::hertz<float> sampleRate, float a0 = 0.5f)
    {
        coefficients.b1 = 0;
        coefficients.a0 = math::clamp<float>(a0, 0, 1);
        coefficients.a1 = 1 - coefficients.a0;
    }
    
    //! Set filter to low pass filtering using one pole, given a samplerate and a cutoff
    template <typename T>
    void lowPassOnePole(FirstOrderCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff)
    {
        // safety check
        FirstOrderCoefficients<T>::check(sampleRate, cutOff);
        
        const auto b1 = exp(-math::TWO_PI<T> * (cutOff.value / sampleRate.value));
        
        coefficients.b1 = -b1; // invert to fit the conventional -b notation in the direct form I
        coefficients.a0 = 1.0 - b1;
        coefficients.a1 = 0;
    }
    
    //! Set filter to low pass filtering using one pole, given a samplerate, time and and a time constant factor.
    /*! @param timeConstantFactor Affects the actual time. A factor of 1 means a step response where the output reaches to ~63% in the given time. A factor of 5 reaches to ~99%. */
    template <typename T>
    void lowPassOnePole(FirstOrderCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::second<float> time, float timeConstantFactor = 5.f)
    {
        // safety check
        FirstOrderCoefficients<T>::check(sampleRate, time, timeConstantFactor);
        
        const auto b1 = exp(-timeConstantFactor / (time.value * sampleRate.value));
        
        coefficients.b1 = -b1; // invert to fit the conventional -b notation in the direct form I
        coefficients.a0 = 1.0 - b1;
        coefficients.a1 = 0;
    }
    
    //! Set filter to low pass filtering using one pole and one zero, given a samplerate and a cutoff
    template <typename T>
    void lowPassOnePoleZero(FirstOrderCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff)
    {
        // safety check
        FirstOrderCoefficients<T>::check(sampleRate, cutOff);
        
        const auto z = std::tan(math::PI<double> * cutOff.value / sampleRate.value);
        const auto s = (z - 1) / (z + 1);
        
        coefficients.b1 = s;
        coefficients.a0 = (1.0 + s) / 2; // or z / (1 + z)
        coefficients.a1 = coefficients.a0;
    }
    
    //! Set filter to low pass filtering using one pole and one zero, given a samplerate, time and and a time constant factor.
    /*! @param timeConstantFactor Affects the actual time. A factor of 1 means a step response where the output reaches to ~63% in the given time. A factor of 5 reaches to ~99%. */
    template <typename T>
    void lowPassOnePoleZero(FirstOrderCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::second<float> time, float timeConstantFactor = 5.f)
    {
        // safety check
        FirstOrderCoefficients<T>::check(sampleRate, time, timeConstantFactor);
        
        const auto normalizedAngularFrequency = timeConstantFactor / (time.value * sampleRate.value);
        
        // return through-pass when normalizedAngularFrequency is higher than pi (nyquist)
        if (normalizedAngularFrequency > math::PI<float>)
            return throughPass(coefficients);
        
        const auto z = std::tan(normalizedAngularFrequency / 2);
        const auto s = (z - 1) / (z + 1);
        
        coefficients.b1 = s;
        coefficients.a0 = (1.0 + s) / 2;
        coefficients.a1 = coefficients.a0;
    }
    
    //! Set filter to high pass filtering using one pole, given a samplerate and a cutoff
    template <typename T>
    void highPassOnePole(FirstOrderCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff)
    {
        // safety check
        FirstOrderCoefficients<T>::check(sampleRate, cutOff);
        
        const auto b1 = exp(-math::TWO_PI<T> * (cutOff.value / sampleRate.value)) - 1;
        
        coefficients.b1 = -b1; // invert to fit the conventional -b notation in the direct form I
        coefficients.a0 = 1.0 + b1;
        coefficients.a1 = 0;
    }
    
    //! Set filter to high pass filtering using one pole and one zero, given a samplerate and a cutoff
    template <typename T>
    void highPassOnePoleZero(FirstOrderCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> cutOff)
    {
        // safety check
        FirstOrderCoefficients<T>::check(sampleRate, cutOff);
        
        const auto z = std::tan(math::PI<double> * cutOff.value / sampleRate.value);
        const auto s = (1 - z) / (z + 1);
        
        coefficients.b1 = -s;
        coefficients.a0 = (1 + s) / 2;
        coefficients.a1 = -coefficients.a0;
    }
    
    //! Set filter to high all-pass filtering using one pole and one zero, given a coefficient value for b1
    template <typename T>
    void allPass(FirstOrderCoefficients<T>& coefficients, float coefficient)
    {
        coefficients.b1 = coefficient;
        coefficients.a0 = coefficient;
        coefficients.a1 = 1;
    }
    
    //! Set filter to high all-pass filtering using one pole and one zero, given a samplerate and a center frequency where the shift is 90 degrees
    template <typename T>
    void allPass(FirstOrderCoefficients<T>& coefficients, unit::hertz<float> sampleRate, unit::hertz<float> centerFrequency)
    {
        // safety check
        FirstOrderCoefficients<T>::check(sampleRate, centerFrequency);
        
        auto z = std::tan(math::PI<double> * (centerFrequency.value / sampleRate.value));
        auto s = (z - 1) / (z + 1);
        
        coefficients.b1 = s;
        coefficients.a0 = s;
        coefficients.a1 = 1;
    }
    
}

#endif /* GRIZZLY_FIRST_ORDER_COEFFICIENTS_HPP */
