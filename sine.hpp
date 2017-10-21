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

#ifndef GRIZZLY_SINE_HPP
#define GRIZZLY_SINE_HPP

#include <cmath>
#include <moditone/math/constants.hpp>

#include "generator.hpp"

namespace dsp
{
    template <typename T, typename Phase>
    constexpr T fastSin(Phase x)
    {
        //always wrap input angle to -PI..PI
        if (x < -math::PI<T>)
            x += math::TWO_PI<T>;
        else if (x >  math::PI<T>)
            x -= math::TWO_PI<T>;
        
        //compute sine
        if (x < 0)
            return 1.27323954 * x + .405284735 * x * x;
        else
            return 1.27323954 * x - 0.405284735 * x * x;
    }
    
    //! Generate a bipolar sine wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateBipolarSine(Phase phase, Phase phaseOffset)
    {
        return std::sin(math::TWO_PI<T> * (phase + phaseOffset));
    }
    
    //! Generate a unipolar sine wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateUnipolarSine(Phase phase, Phase phaseOffset)
    {
        return generateBipolarSine<T>(phase - 0.25f + phaseOffset) * 0.5 + 0.5;
    }
    
    template <typename T>
    class Sine :
        public Generator<T>
    {
    public:
        using Generator<T>::Generator;
        
        T convert() final
        {
            return generateBipolarSine<T>(this->getPhase(), this->getPhaseOffset());
        }
    };
    
    //! Generates a bipolar sine wave using the polyBLEP algorithm for anti aliasing when synced
    template <typename T>
    class BandLimitedSine :
        public BandLimitedGenerator<T>
    {
    public:
        using BandLimitedGenerator<T>::BandLimitedGenerator;
        
    private:
        T computeAliasedY() final
        {
            return dsp::generateBipolarSine<T>(this->getPhase(), this->getPhaseOffset());
        }
        
        void applyRegularBandLimiting(T& y) final
        {
            
        }
        
        T computeAliasedYBeforeReset(long double phase, long double phaseOffset) final
        {
            return generateBipolarSine<T>(phase, phaseOffset);
        }
        
        T computeAliasedYAfterReset(long double phase, long double phaseOffset) final
        {
            return generateBipolarSine<T>(phase, phaseOffset);
        }
    };
}

#endif /* GRIZZLY_SINE_HPP */

