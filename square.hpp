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

#include "generator.hpp"
#include "poly_blep.hpp"

namespace dsp
{
    //! Generate a square wave given a normalized phase
    template <typename T, typename Phase, typename PulseWidth>
    constexpr T generateSquare(Phase phase, Phase phaseOffset, PulseWidth pulseWidth, const T& low, const T& high) noexcept
    {
        return math::wrap<Phase>(phase + phaseOffset, 0, 1) < pulseWidth ? high : low;
    }
    
    //! Generates a bipolar square wave
    template <typename T>
    class Square :
        public Generator<T>
    {
    public:
        //! Change the pulse width
        /*! @param recompute Recompute the y to return from read() */
        void setPulseWidth(float pulseWidth, bool recompute)
        {
            if (pulseWidth == this->pulseWidth)
                return;
            
            this->pulseWidth = pulseWidth;
            
            if (recompute)
                this->recompute();
        }
        
    private:
        //! Recompute the most recently computed value
        T convert() final { return generateSquare<T>(this->getPhase(), this->getPhaseOffset(), pulseWidth, -1, 1); }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
    
    //! Generates a bipolar square wave using the polyBLEP algorithm for anti aliasing
    template <typename T>
    class BandLimitedSquare :
        public BandLimitedGenerator<T>
    {
    public:
        using BandLimitedGenerator<T>::BandLimitedGenerator;
        
        //! Change the pulse width
        /*! @param recompute Recompute the y to return from read() */
        void setPulseWidth(float pulseWidth, bool recompute)
        {
            if (pulseWidth == this->pulseWidth)
                return;
            
            this->pulseWidth = pulseWidth;
            
            if (recompute)
                this->recompute();
        }
        
    private:
        T computeAliasedY(const long double& phase, const long double& phaseOffset) noexcept final
        {
            return generateSquare<T>(phase, phaseOffset, pulseWidth, -1, 1);
        }
        
        void applyRegularBandLimiting(const long double& phase_, const long double& phaseOffset, const long double& increment, T& y) noexcept final
        {
            const auto phase = phase_ + phaseOffset;

            y += polyBlep<long double>(math::wrap<long double>(phase, 0.0, 1.0), increment);
            y -= polyBlep<long double>(math::wrap<long double>(phase + (1 - pulseWidth), 0, 1), increment);
        }
        
        T computeAliasedYBeforeReset(long double phase, long double phaseOffset) noexcept final
        {
            return generateSquare<T>(phase, phaseOffset, pulseWidth, -1.0l, 1.0l);
        }
        
        T computeAliasedYAfterReset(long double phase, long double phaseOffset) noexcept final
        {
            return generateSquare<T>(phase, phaseOffset, pulseWidth, -1.0l, 1.0l);
        }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
}

#endif /* GRIZZLY_SQUARE_HPP */

