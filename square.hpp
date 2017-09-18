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

#ifndef GRIZZLY_SQUARE_HPP
#define GRIZZLY_SQUARE_HPP

#include <moditone/math/wrap.hpp>

#include "phase_generator.hpp"
#include "poly_blep.hpp"

namespace dsp
{
    //! Generate a square wave given a normalized phase
    template <typename T, typename Phase, typename PulseWidth>
    constexpr T generateSquare(Phase phase, PulseWidth pulseWidth, const T& low, const T& high)
    {
        return math::wrap<Phase>(phase, 0, 1) < pulseWidth ? high : low;
    }
    
    //! Generates a bipolar square wave
    template <typename T>
    class BipolarSquare : public PhaseGenerator<T>
    {
    public:
        BipolarSquare(float pulseWidth = 0.5f) :
            pulseWidth(pulseWidth)
        {
            
        }
        
        //! Change the pulse width
        /*! @param recompute Recompute the y to return from read() */
        void setPulseWidth(float pulseWidth, bool recompute)
        {
            if (pulseWidth == this->pulseWidth)
                return;
            
            this->pulseWidth = pulseWidth;
            
            if (recompute)
                this->recomputeY();
        }
        
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY(long double phase) final { return dsp::generateSquare<T>(phase, pulseWidth, -1, 1); }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
    
    //! Generates a bipolar square wave
    template <typename T>
    class UnipolarSquare : public PhaseGenerator<T>
    {
    public:
        UnipolarSquare(float pulseWidth = 0.5f) :
            pulseWidth(pulseWidth)
        {
        }
        
        //! Change the pulse width
        /*! @param recompute Recompute the y to return from read() */
        void setPulseWidth(float pulseWidth, bool recompute)
        {
            if (pulseWidth == this->pulseWidth)
                return;
            
            this->pulseWidth = pulseWidth;
            
            if (recompute)
                this->recomputeY();
        }
        
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY(long double phase) final { return dsp::generateSquare<T>(phase, pulseWidth, 0, 1); }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
    
    //! Generates a bipolar square wave
    template <typename T>
    class BipolarSquareBlep : public PhaseGenerator<T>
    {
    public:
        BipolarSquareBlep(float pulseWidth = 0.5f) :
            pulseWidth(pulseWidth)
        {
        }
        
        //! Change the pulse width
        /*! @param recompute Recompute the y to return from read() */
        void setPulseWidth(float pulseWidth, bool recompute)
        {
            if (pulseWidth == this->pulseWidth)
                return;
            
            this->pulseWidth = pulseWidth;
            
            if (recompute)
                this->recomputeY();
        }
        
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY(long double phase) final
        {
            // Compute the y without any anti aliasing
            auto y = dsp::generateSquare<T>(phase, pulseWidth, -1, 1);
            
            // Compute the increment (phase - previous) and adjust y using polyBLEP
            const auto increment = phase - previousPhase;
            y += polyBlep<long double>(phase, increment);
            y -= polyBlep<long double>(math::wrap<long double>(this->getPhase() + (1 - pulseWidth), 0, 1), increment);
            
            // Update the previous phase
            previousPhase = phase;
            
            return y;
        }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
        
        long double previousPhase = 0;
    };
}

#endif /* GRIZZLY_SQUARE_HPP */
