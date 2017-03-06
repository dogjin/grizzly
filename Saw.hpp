/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2017 Dsperados <info@dsperados.com>
 
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

#ifndef GRIZZLY_SAW_HPP
#define GRIZZLY_SAW_HPP

#include <dsperados/math/utility.hpp>
#include <unit/hertz.hpp>

namespace dsp
{
    //! Generate a bipolar saw wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateBipolarSaw(Phase phase)
    {
        return math::wrap<std::common_type_t<Phase, T>>(phase + 0.5, 0, 1) * 2 - 1;
    }
    
    //! Generate a unipolar saw wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateUnipolarSaw(Phase phase)
    {
        return math::wrap<std::common_type_t<Phase, T>>(phase, 0, 1);
    }
    
    //! Generates a saw wave
    template <typename T>
    class BipolarSaw
    {
    public:
        //! Increment the phase of the saw
        void increment(long double increment)
        {
            setPhase(phase + increment);
            recomputeY();
        }
        
        //! Increment the phase, given a frequency
        void increment(unit::hertz<float> frequency, unit::hertz<float> sampleRate)
        {
            increment(frequency.value / sampleRate.value);
        }
        
        //! Read the most recently computed output
        T read() const
        {
            return y;
        }
        
        //! Change the phase manually
        void setPhase(long double phase)
        {
            this->phase = math::wrap<long double>(phase, 0, 1);
            recomputeY();
        }
        
    private:
        //! Recompute the most recently computed value
        void recomputeY()
        {
            y = dsp::generateBipolarSaw<T>(phase);
        }
        
    private:
        //! The to be returned value from read
        T y;
        
        //! The current phase of the saw (range 0-1)
        long double phase = 0;
    };
    
    //! Generates a saw wave
    template <typename T>
    class UnipolarSaw
    {
    public:
        //! Increment the phase of the saw
        void increment(long double increment)
        {
            setPhase(phase + increment);
            recomputeY();
        }
        
        //! Increment the phase, given a frequency
        void increment(unit::hertz<float> frequency, unit::hertz<float> sampleRate)
        {
            increment(frequency.value / sampleRate.value);
        }
        
        //! Read the most recently computed output
        T read() const
        {
            return y;
        }
        
        //! Change the phase manually
        void setPhase(long double phase)
        {
            this->phase = math::wrap<long double>(phase, 0, 1);
            recomputeY();
        }
        
    private:
        //! Recompute the most recently computed value
        void recomputeY()
        {
            y = dsp::generateUnipolarSaw<T>(phase);
        }
        
    private:
        //! The to be returned value from read
        T y;
        
        //! The current phase of the saw (range 0-1)
        long double phase = 0;
    };
}

#endif /* GRIZZLY_SAW_HPP */
