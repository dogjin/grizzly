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

#include <dsperados/math/interpolation.hpp>

#include "phase_generator.hpp"
#include "poly_blep.hpp"

namespace dsp
{
    //! Generate a bipolar saw wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateBipolarSaw(Phase phase)
    {
        return math::wrap<std::common_type_t<Phase, T>>(phase + 0.5, 0, 1) * 2 - 1;
    }
    
    //! Generate a unipolar saw wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateUnipolarSaw(Phase phase)
    {
        return math::wrap<std::common_type_t<Phase, T>>(phase, 0, 1);
    }
    
    //! Generates a bipolar saw wave
    template <typename T>
    class BipolarSaw : public PhaseGenerator<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY(long double phase) final override { return dsp::generateBipolarSaw<T>(phase); }
    };
    
    //! Generates a unipolar saw wave
    template <typename T>
    class UnipolarSaw : public PhaseGenerator<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY(long double phase) final override { return dsp::generateUnipolarSaw<T>(phase); }
    };
    
    //! Generates a bipolar saw wave using the polyBLEP algorithm for anti aliasing
    template <typename T>
    class BipolarSawBlep : public PhaseGenerator<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY(long double phase) final override
        {
            // Compute the y without any anti aliasing
            auto y = dsp::generateBipolarSaw<T>(phase);
            
            // Compute the increment (phase - previous) and adjust y using polyBLEP
            y -= polyBlep<long double>(math::wrap<T>(this->getPhase() + 0.5, 0.0, 1.0), phase - previousPhase);
            
            // Update the previous phase
            previousPhase = phase;
            
            return y;
        }
        
    private:
        long double previousPhase = 0;
    };
}

#endif /* GRIZZLY_SAW_HPP */
