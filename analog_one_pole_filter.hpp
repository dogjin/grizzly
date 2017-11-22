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

#ifndef GRIZZLY_ANALOG_ONE_POLE_FILTER_HPP
#define GRIZZLY_ANALOG_ONE_POLE_FILTER_HPP

#include <cmath>
#include <moditone/math/constants.hpp>
#include <functional>
#include <stdexcept>
#include <moditone/unit/hertz.hpp>
#include <moditone/unit/time.hpp>

namespace dsp
{
    //! Topology preserving one pole filter with resolved zero feedback delay
    /*! See "The Art Of VA Filter Design" by Vadim Zavalishin. */
    template <class T>
    class AnalogOnePoleFilter
    {
    public:
        //! Write a sample to the filter
        void write(const T& x)
        {
            const auto integratorInput = (x - integratorState) * cutOffGain;
            
            // write low-pass state
            lowPassOutputState = integratorInput + integratorState;
            
            // write high-pass state
            highPassOutputState = x - lowPassOutputState;
            
            // write integrator state for next call
            integratorState = lowPassOutputState + integratorInput;
            
            // Optional non-linear processing
            if (nonLinear)
                integratorState = nonLinear(integratorState);
        }
        
        //! Read the low-pass output
        T readLowPass() const
        {
            return lowPassOutputState;
        }
        
        //! Read the high-pass output
        T readHighPass() const
        {
            return highPassOutputState;
        }
        
        //! Write and read low-pass output (in that order)
        T writeAndReadLowPass(const T& x)
        {
            write(x);
            return readLowPass();
        }
        
        //! Write and read high-pass output (in that order)
        T writeAndReadHighPass(const T& x)
        {
            write(x);
            return readHighPass();
        }
        
        //! Set cut-off
        void setCutOff(unit::hertz<float> cutOff, unit::hertz<float> sampleRate)
        {
            // check sample rate
            if (sampleRate.value <= 0)
                throw std::invalid_argument("sampling rate <= 0");
            
            // check cut-off
            const auto nyquist = sampleRate.value / 2;
            if (cutOff.value <= 0 || cutOff.value >= nyquist)
                throw std::invalid_argument("cut-off <= 0 or >= nyquist");
            
            auto unresolvedCutOffGain = std::tan(math::PI<T> * cutOff.value / sampleRate.value);
            cutOffGain = unresolvedCutOffGain / (1.0f + unresolvedCutOffGain);
        }
        
        //! Set time with a default time-constant-factor
        /*! @param timeConstantFactor Affects the actual time. A factor of 1 means a step response where the output reaches to ~63% in the given time. A factor of 5 reaches to ~99%. */
        void setTime(unit::second<float> time, unit::hertz<float> sampleRate, float timeConstantFactor = 5.f)
        {
            // check sample rate
            if (sampleRate.value <= 0)
                throw std::invalid_argument("sampling rate <= 0");
            
            // check time
            if (time.value <= 0)
                throw std::invalid_argument("time <= 0");
            
            // check time-constant-filter
            if (timeConstantFactor < 0)
                throw std::invalid_argument("time constant factor < 0");
            
            auto unresolvedCutOffGain = std::tan(timeConstantFactor / (time.value * sampleRate.value * 2));
            cutOffGain = unresolvedCutOffGain / (1.0 + unresolvedCutOffGain);
        }
        
        //! Set cut-off gain directly, useful when creating more complex filter structures (use with caution)
        void setCutOffGain(T cutOffGain)
        {
            this->cutOffGain = cutOffGain;
        }
        
        //! Set the filter state to a value directly
        void setState(T state)
        {
            integratorState = state;
            lowPassOutputState = state;
            highPassOutputState = 0;
        }
        
        //! Reset the filter to zero
        void reset()
        {
            setState(0);
        }
        
        //! Get the integrator state, useful when creating more complex filter structures
        T getIntegratorState() const { return integratorState; }
        
    public:
        //! Function for non-linear processing
        std::function<T(const T&)> nonLinear;
        
    private:
        //! Filter gain factor with resolved zero delay feedback
        T cutOffGain = 0;
        
        //! Integrator state
        T integratorState = 0;
        
        //! Low-pass output state
        T lowPassOutputState = 0;
        
        //! Low-pass output state
        T highPassOutputState = 0;
    };
}

#endif
