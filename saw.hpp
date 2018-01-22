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
    template <typename T>
    constexpr T generateBipolarSaw(T phase, T phaseOffset) noexcept
    {
        return math::wrap<T>(phase + phaseOffset, T(0), T(1)) * T(2) - T(1);
    }
    
    //! Generate a unipolar saw wave given a normalized phase
    template <typename T>
    constexpr T generateUnipolarSaw(T phase, T phaseOffset) noexcept
    {
        return math::wrap<T>(phase + phaseOffset, T(0), T(1));
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
        public BandLimitedGenerator<T, BandLimitedSaw<T>>
    {
    public:
        using BandLimitedGenerator<T, BandLimitedSaw<T>>::BandLimitedGenerator;
        
        T computeAliasedY(const long double& phase, const long double& phaseOffset) const noexcept
        {
            return up ? generateBipolarSaw<T>(phase, phaseOffset) : generateBipolarSaw<T>(phase, phaseOffset) * -1;
        }
        
        void applyRegularBandLimiting(const long double& phase, const long double& phaseOffset, const long double& increment, T& y) noexcept
        {
            up ?
            y -= polyBlep<long double>(math::wrap<long double>(phase + phaseOffset, 0.0, 1.0), increment) :
            y += polyBlep<long double>(math::wrap<long double>(phase + phaseOffset, 0.0, 1.0), increment);
        }
        
    public:
        bool up = true;
    };
    
    template <typename T>
    class BandLimitedSawUnipolar :
        public BandLimitedGenerator<T, BandLimitedSawUnipolar<T>>
    {
    public:
        using BandLimitedGenerator<T, BandLimitedSawUnipolar<T>>::BandLimitedGenerator;
        
        T computeAliasedY(const long double& phase, const long double& phaseOffset) const noexcept
        {
            return up ? generateUnipolarSaw<T>(phase, phaseOffset) : generateUnipolarSaw<T>(phase, phaseOffset) * -1 + 1;
        }
        
        void applyRegularBandLimiting(const long double& phase, const long double& phaseOffset, const long double& increment, T& y) noexcept
        {
            up ?
            y -= (polyBlep<long double>(math::wrap<long double>(phase + phaseOffset, 0.0, 1.0), increment) / 2.l) :
            y += (polyBlep<long double>(math::wrap<long double>(phase + phaseOffset, 0.0, 1.0), increment) / 2.l);
        }
        
    public:
        bool up = true;
    };
}

#endif /* GRIZZLY_SAW_HPP */

