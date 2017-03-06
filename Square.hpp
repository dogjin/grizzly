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

#ifndef GRIZZLY_SQUARE_HPP
#define GRIZZLY_SQUARE_HPP

#include <dsperados/math/utility.hpp>
#include <unit/hertz.hpp>

namespace dsp
{
    //! Generate a square wave given a normalized phase
    template <typename T, typename Phase, typename PulseWidth>
    constexpr T generateSquare(Phase phase, PulseWidth pulseWidth, T low = 0, T high = 1)
    {
        return math::wrap<Phase>(phase, 0, 1) < pulseWidth ? high : low;
    }
    
    //! Generates a square wave
    template <typename T>
    class Square
    {
    public:
        Square(const T& min = -1, const T& max = 1) :
            min(min),
            max(max)
        {
            
        }
        
        //! Increment the phase of the square
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
        
        //! Change the pulse width
        /*! @param recompute: Recompute the y to return from read() */
        void setPulseWidth(float pulseWidth, bool recompute)
        {
            this->pulseWidth = pulseWidth;
            
            if (recompute)
                recomputeY();
        }
        
    private:
        //! Recompute the most recently computed value
        void recomputeY()
        {
            y = dsp::generateSquare<T>(phase, pulseWidth);
        }
        
    private:
        //! The to be returned value from read
        T y;
        
        //! The minimum value
        T min = -1;
        
        //! The maximum value;
        T max = 1;
        
        //! The current phase of the square (range 0-1)
        long double phase = 0;
        
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
}

#endif /* GRIZZLY_SQUARE_HPP */
