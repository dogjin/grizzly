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

#ifndef GRIZZLY_SAW_HPP
#define GRIZZLY_SAW_HPP

#include <cassert>
#include <cmath>
#include <memory>
#include <moditone/math/interpolation.hpp>
#include <type_traits>

#include "generator.hpp"
#include "poly_blep.hpp"

namespace dsp
{
    //! Generate a bipolar saw wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateBipolarSaw(Phase phase, Phase phaseOffset)
    {
        return math::wrap<std::common_type_t<Phase, T>>(phase + phaseOffset, 0, 1) * 2 - 1;
    }
    
    //! Generate a unipolar saw wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateUnipolarSaw(Phase phase, Phase phaseOffset)
    {
        return math::wrap<std::common_type_t<Phase, T>>(phase + phaseOffset, 0, 1);
    }
    
    template <typename T>
    class Saw :
        public Generator<T>
    {
    public:
        using Generator<T>::Generator;
        
        T convert() final
        {
            return generateBipolarSaw<T>(this->getPhase(), this->getPhaseOffset());
        }
    };
    
    template <typename T>
    class BandLimitedSaw :
        public BandLimitedGenerator<T>
    {
    public:
        using BandLimitedGenerator<T>::BandLimitedGenerator;
        
    private:
        T computeAliasedY() final
        {
            return generateBipolarSaw<T>(this->getPhase(), this->getPhaseOffset());
        }
        
        void applyRegularBandLimiting(T& y) final
        {
            y -= polyBlep<long double>(math::wrap<long double>(this->getPhase() + this->getPhaseOffset(), 0.0, 1.0), this->getIncrement());
        }
        
        T computeAliasedYBeforeReset(long double phase, long double phaseOffset) final
        {
            return generateBipolarSaw<T>(phase, phaseOffset);
        }
        
        T computeAliasedYAfterReset(long double phase, long double phaseOffset) final
        {
            return generateBipolarSaw<T>(phase, phaseOffset);
        }
    };
}

#endif /* GRIZZLY_SAW_HPP */

