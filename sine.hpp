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

#include "phase_generator.hpp"

namespace dsp
{
    //! Generate a bipolar sine wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateBipolarSine(Phase phase)
    {
        return std::sin(math::TWO_PI<T> * phase);
    }
    
    //! Generate a unipolar sine wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateUnipolarSine(Phase phase)
    {
        return generateBipolarSine<T>(phase - 0.25f) * 0.5 + 0.5;
    }
    
    //! Generates a bipolar sine wave
    /*! For fast sine wave approximation, use the Gordon-Smith oscillator */
    template <typename T>
    class BipolarSine : public PhaseGenerator<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY(long double phase) final override { return dsp::generateBipolarSine<T>(phase); }
    };
    
    //! Generates a unipolar sine wave
    template <typename T>
    class UnipolarSine : public PhaseGenerator<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY(long double phase) final override { return dsp::generateUnipolarSine<T>(phase); }
    };
}

#endif /* GRIZZLY_SINE_HPP */
