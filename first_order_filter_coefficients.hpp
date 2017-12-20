/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/moditone/grizzly for more information.
 
 Copyright (C) 2016-2018 Moditone <info@moditone.com>
 
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

#include <moditone/math/constants.hpp>
#include <moditone/math/clamp.hpp>

namespace dsp
{
    //! Coefficients for a first-order, one-pole/one-zero filter
    template <class T>
    struct FirstOrderCoefficients
    {
        //! The a0 feed-forward coefficient (gain)
        T a0 = 1;
        
        //! The a1 feed-forward coefficient
        T a1 = 0;
        
        //! The b1 feed-back coefficient
        T b1 = 0;
        
        //! Check if the pole z (-b1) stays within the unit bounds -1, 1
        bool isStable() const
        {
            if (b1 > -1 && b1 < 1)
                return true;
            else
                return false;
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
    void lowPassOneZero(FirstOrderCoefficients<T>& coefficients, float a0 = 0.5f)
    {
        coefficients.b1 = 0;
        coefficients.a0 = math::clamp<float>(a0, 0, 1);
        coefficients.a1 = 1 - coefficients.a0;
    }
    
    //! Set filter to low pass filtering using one pole, given a sampleRate and a cutOff
    template <typename T>
    void lowPassOnePole(FirstOrderCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz)
    {
        const T b1 = -std::exp(-math::TWO_PI<T> * (cutOff_Hz / sampleRate_Hz));
        
        coefficients.b1 = b1; // invert to fit the conventional -b notation in the direct form I
        coefficients.a0 = 1.0f + b1;
        coefficients.a1 = 0.f;
    }
    
    //! Set filter to low pass filtering using one pole, given a sampleRate, time and and a time constant factor.
    /*! @param timeConstantFactor Affects the actual time. A factor of 1 means a step response where the output reaches to ~63% in the given time. A factor of 5 reaches to ~99%. */
    template <typename T>
    void lowPassOnePole(FirstOrderCoefficients<T>& coefficients, T sampleRate_Hz, T time_s, T timeConstantFactor)
    {
        const T b1 = -std::exp(-timeConstantFactor / (time_s * sampleRate_Hz));
        
        coefficients.b1 = b1; // invert to fit the conventional -b notation in the direct form I
        coefficients.a0 = 1.0f + b1;
        coefficients.a1 = 0.f;
    }
    
    //! Set filter to low pass filtering using one pole and one zero, given a sampleRate and a cutOff
    template <typename T>
    void lowPassOnePoleZero(FirstOrderCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz)
    {
        const auto z = std::tan(math::PI<T> * (cutOff_Hz / sampleRate_Hz));
        const auto s = (z - 1) / (z + 1);
        
        coefficients.b1 = s;
        coefficients.a0 = (1.0 + s) / 2; // same as z / (1 + z)
        coefficients.a1 = coefficients.a0;
    }
    
    //! Set filter to low pass filtering using one pole and one zero, given a sampleRate, time and and a time constant factor.
    /*! @param timeConstantFactor Affects the actual time. A factor of 1 means a step response where the output reaches to ~63% in the given time. A factor of 5 reaches to ~99%. */
    template <typename T>
    void lowPassOnePoleZero(FirstOrderCoefficients<T>& coefficients, T sampleRate_Hz, T time_s, T timeConstantFactor)
    {
        const auto z = std::tan(timeConstantFactor / (time_s * sampleRate_Hz * 2));
        const auto s = (z - 1) / (z + 1);
        
        coefficients.b1 = s;
        coefficients.a0 = (1.0 + s) / 2; // same as z / (1 + z)
        coefficients.a1 = coefficients.a0;
    }
    
    //! Set filter to high pass filtering using one pole, given a sampleRate and a cutOff
    template <typename T>
    void highPassOnePole(FirstOrderCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz)
    {
        const auto b1 = 1 - std::exp(-math::TWO_PI<T> * (cutOff_Hz / sampleRate_Hz));
        
        coefficients.b1 = b1; // invert to fit the conventional -b notation in the direct form I
        coefficients.a0 = 1.0 - b1;
        coefficients.a1 = 0;
    }
    
    //! Set filter to high pass filtering using one pole and one zero, given a sampleRate and a cutOff
    template <typename T>
    void highPassOnePoleZero(FirstOrderCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz)
    {
        const auto z = std::tan(math::PI<double> * (cutOff_Hz / sampleRate_Hz));
        const auto s = (1 - z) / (z + 1);
        
        coefficients.b1 = -s;
        coefficients.a0 = (1 + s) / 2;
        coefficients.a1 = -coefficients.a0;
    }
    
    //! Set filter to high all-pass filtering using one pole and one zero, given a shape value for the a0 and b1 coefficients
    template <typename T>
    void allPass(FirstOrderCoefficients<T>& coefficients, T shape)
    {
        coefficients.b1 = -shape;
        coefficients.a0 = -shape;
        coefficients.a1 = 1;
    }
    
    //! Set filter to high all-pass filtering using one pole and one zero, given a sampleRate and a center frequency where the shift is 90 degrees
    template <typename T>
    void allPass(FirstOrderCoefficients<T>& coefficients, T sampleRate_Hz, T centerFrequency)
    {
        const auto z = std::tan(math::PI<T> * (centerFrequency / sampleRate_Hz));
        const auto s = (z - 1) / (z + 1);
        
        coefficients.b1 = s;
        coefficients.a0 = s;
        coefficients.a1 = 1;
    }
    
}

#endif /* GRIZZLY_FIRST_ORDER_COEFFICIENTS_HPP */
