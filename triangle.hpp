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

#ifndef GRIZZLY_TRIANGLE_HPP
#define GRIZZLY_TRIANGLE_HPP

#include <cassert>
#include <cmath>
#include <memory>
#include <moditone/math/wrap.hpp>

#include "poly_blamp.hpp"
#include "phase_generator.hpp"

namespace dsp
{
    //! Generate a unipolar triangle wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateUnipolarTriangle(Phase phase, Phase phaseOffset)
    {
        phase = math::wrap<Phase>(phase + phaseOffset, 0, 1);
        return phase < 0.5 ? phase * 2 : (0.5 - (phase - 0.5)) * 2;
    }
    
    //! Generate a bipolar triangle wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateBipolarTriangle(Phase phase, Phase phaseOffset)
    {
        return generateUnipolarTriangle<T>(phase, phaseOffset) * 2 - 1;
        //        return 2 * std::fabs(2 * math::wrap<std::common_type_t<Phase, T>>(phase + phaseOffset - 0.25, 0, 1) - 1) - 1;
    }
    
    //! Generates a bipolar triangle wave
    template <typename T>
    class BipolarTriangle : public Phasor<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY() final { return dsp::generateBipolarTriangle<T>(this->phase, this->phaseOffset); }
    };
    
    //! Generates a unipolar triangle wave
    template <typename T>
    class UnipolarTriangle : public Phasor<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY() final { return dsp::generateUnipolarTriangle<T>(this->phase, this->phaseOffset); }
    };
    
    //! Generates a bipolar triangle wave using the polyBLEP algorithm for anti aliasing
    template <typename T>
    class BipolarTriangleBlamp : public PhasorBlep<T>
    {
    private:        
        T computeAliasedY() final
        {
            return dsp::generateBipolarTriangle<T>(this->phase, this->phaseOffset);
        }
        
        void applyRegularBandLimiting(T& y) final
        {
            // Downward
            auto scale = 4 * this->increment_;
            auto modifiedPhase = this->phase;//this->phase + 0.25;
            modifiedPhase -= floor(modifiedPhase);
            y += scale * polyBlamp(modifiedPhase, this->increment_);
            
            // Upward
            modifiedPhase += 0.5;
            modifiedPhase -= floor(modifiedPhase);
            y -= scale * polyBlamp(modifiedPhase, this->increment_);
        }
        
        T computeAliasedYBeforeReset(long double phase, long double phaseOffset) final
        {
            return generateBipolarTriangle<T>(phase, phaseOffset);
        }
        
        T computeAliasedYAfterReset(long double phase, long double phaseOffset) final
        {
            return generateBipolarTriangle<T>(phase, phaseOffset);
        }
    };
}

#endif /* GRIZZLY_TRIANGLE_HPP */
