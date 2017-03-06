/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2016 Dsperados <info@dsperados.com>
 
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

#ifndef GRIZZLY_PHASE_GENERATOR_HPP
#define GRIZZLY_PHASE_GENERATOR_HPP

#include <dsperados/math/utility.hpp>
#include <unit/hertz.hpp>

namespace dsp
{
    //! Generates a waveform using incrementable phase
    template <typename T>
    class PhaseGenerator
    {
    public:
        //! Increment the generator
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
        
        //! Return the current phase between 0 and 1
        long double getPhase() const
        {
            return phase;
        }
        
    protected:
        //! Recompute the y to be returned
        void recomputeY() { y = convertPhaseToY(phase); }
        
    private:
        //! Recompute the most recently computed value
        virtual T convertPhaseToY(long double phase) const = 0;
        
    private:
        //! The to be returned value from read
        T y;
        
        //! The current phase of the saw (ranged from 0 to 1)
        long double phase = 0;
    };
}

#endif /* GRIZZLY_PHASE_GENERATOR_HPP */
