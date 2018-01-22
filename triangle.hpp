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

#ifndef GRIZZLY_TRIANGLE_HPP
#define GRIZZLY_TRIANGLE_HPP

#include <cassert>
#include <cmath>
#include <memory>
#include <moditone/math/wrap.hpp>

#include "generator.hpp"
#include "poly_blamp.hpp"

namespace dsp
{
    //! Generate a unipolar triangle wave given a normalized phase
    template <typename T>
    constexpr T generateUnipolarTriangle(T phase, T phaseOffset) noexcept
    {
        phase = math::wrap<T>(phase + phaseOffset, 0, 1);
        return phase < 0.5 ? phase * 2 : (0.5 - (phase - 0.5)) * 2;
    }
    
    //! Generate a bipolar triangle wave given a normalized phase
    template <typename T>
    constexpr T generateBipolarTriangle(T phase, T phaseOffset) noexcept
    {
        return generateUnipolarTriangle<T>(phase + 0.25, phaseOffset) * 2 - 1;
    }
    
    template <typename T>
    class Triangle :
        public Generator<T>
    {
    public:
        using Generator<T>::Generator;
        
        T convert() final
        {
            return generateBipolarTriangle<T>(this->getPhase(), this->getPhaseOffset());
        }
    };
        
    template <typename T>
    class TriangleUnipolar :
    public Generator<T>
    {
    public:
        using Generator<T>::Generator;
        
        T convert() final
        {
            return generateUnipolarTriangle<T>(this->getPhase(), this->getPhaseOffset());
        }
    };
    
    //! Generates a bipolar triangle wave using the polyBLAMP algorithm for anti aliasing
    template <typename T>
    class BandLimitedTriangle :
        public BandLimitedGenerator<T, BandLimitedTriangle<T>>
    {
    public:
        using BandLimitedGenerator<T, BandLimitedTriangle<T>>::BandLimitedGenerator;
        
        T computeAliasedY(const double& phase, const double& phaseOffset) const noexcept
        {
            return generateBipolarTriangle<T>(phase, phaseOffset);
        }
        
        void applyRegularBandLimiting(const double& phase, const double& phaseOffset, const double& increment, T& y) noexcept
        {            
            // Downward
            const auto scale = 4 * increment;
            // add 0.25 when starting the wave from 0
            auto modifiedPhase = math::wrap<double>(phase + phaseOffset + 0.25, 0.0, 1.0); // this->phase + 0.25; als we de triangle off-setten, dan hier ook!
            modifiedPhase -= floor(modifiedPhase);
            const auto blamp = polyBlamp(modifiedPhase, increment);
            y += scale * blamp;
            
            // If there is a value for the downward blamp, upward can't take place
            if (blamp)
                return;
            
            // Upward
            modifiedPhase += 0.5;
            modifiedPhase -= floor(modifiedPhase);
            y -= scale * polyBlamp(modifiedPhase, increment);
        }
    };
}

#endif /* GRIZZLY_TRIANGLE_HPP */
