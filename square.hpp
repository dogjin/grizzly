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

#include <cassert>
#include <cmath>
#include <memory>
#include <moditone/math/wrap.hpp>

#include "phase_generator.hpp"
#include "poly_blep.hpp"

namespace dsp
{
    //! Generate a square wave given a normalized phase
    template <typename T, typename Phase, typename PulseWidth>
    constexpr T generateSquare(Phase phase, Phase phaseOffset, PulseWidth pulseWidth, const T& low, const T& high)
    {
        return math::wrap<Phase>(phase + phaseOffset, 0, 1) < pulseWidth ? high : low;
    }
    
    //! Generates a bipolar square wave
    template <typename T>
    class BipolarSquare : public Phasor<T>
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
        T convertPhaseToY() final { return dsp::generateSquare<T>(this->phase, this->phaseOffset, pulseWidth, -1, 1); }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
    
    //! Generates a bipolar square wave
    template <typename T>
    class UnipolarSquare : public Phasor<T>
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
        T convertPhaseToY() final { return dsp::generateSquare<T>(this->phase, this->phaseOffset, pulseWidth, 0, 1); }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
    
    
    //! Generates a bipolar saw wave using the polyBLEP algorithm for anti aliasing
    template <typename T>
    class BipolarSquareBlep : public PhasorBlep<T>
    {
    public:
        BipolarSquareBlep() = default;
        
        BipolarSquareBlep(float pulseWidth) : pulseWidth(pulseWidth)
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
                this->convertPhaseToY();
        }
        
    private:
        T computeAliasedY() final
        {
            return dsp::generateSquare<T>(this->phase, this->phaseOffset, pulseWidth, -1, 1);
        }
        
        void applyRegularBandLimiting(T& y) final
        {
            y += polyBlep<long double>(math::wrap<long double>(this->getPhase() + this->phaseOffset, 0.0, 1.0), this->increment_);
            y -= polyBlep<long double>(math::wrap<long double>(this->getPhase() + this->phaseOffset + (1 - pulseWidth), 0, 1), this->increment_);
        }
        
        T computeAliasedYBeforeReset(long double phase, long double phaseOffset) final
        {
            return generateSquare<T>(phase, phaseOffset, pulseWidth, -1.0l, 1.0l);
        }
        
        T computeAliasedYAfterReset(long double phase, long double phaseOffset) final
        {
            return generateSquare<T>(phase, phaseOffset, pulseWidth, -1.0l, 1.0l);
        }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
}

#endif /* GRIZZLY_SQUARE_HPP */

