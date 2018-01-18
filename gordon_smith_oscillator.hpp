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

#ifndef GRIZZLY_GORDON_SMITH_OSCILLATOR_HPP
#define GRIZZLY_GORDON_SMITH_OSCILLATOR_HPP

#include <cmath>

#include <moditone/math/constants.hpp>


namespace dsp
{
    //! Sine approximation using Gordon-Smith
    /*  TODO: Add/replace interface for setting the frequency */
    template <class T>
    class GordonSmithOscillator
    {
    public:
        //! Construct the oscillator
        constexpr GordonSmithOscillator() = default;
        
        //! Construct the oscillator
        constexpr GordonSmithOscillator(T angle_rad)
        {
            setAngle(angle_rad);
        }
        
        //! Compute the next sample
        constexpr T process()
        {
            yq -= epsilon * y;
            
            return y += epsilon * yq;
        }
        
        //! Compute the next sample
        constexpr T operator()()
        {
            return process();
        }
        
        //! Change the angle (in radians)
        constexpr void setAngle(T angle_rad)
        {
            epsilon = 2 * std::sin(angle_rad / 2);
            y = std::sin(-angle_rad);
            yq = std::cos(-angle_rad);
        }
        
    private:
        //! The last calculated y
        T y = 0;
        
        //! The last calculaced quadratic y (90 degrees from y)
        T yq = 0;
        
        //! The Gordon-Smith epsilon = 2 * sin(theta / 2)
        T epsilon = 0;
    };
}

#endif /* GRIZZLY_GORDON_SMITH_OSCILLATOR_HPP */
