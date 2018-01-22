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
 along with this program. If not, see <http://www.gnu.org/licenses/>
 
 --------------------------------------------------------------------
 
 If you would like to use Grizzly for commercial or closed-source
 purposes, please contact us for a commercial license.
 
 */

#ifndef GRIZZLY_POLY_BLAMP_HPP
#define GRIZZLY_POLY_BLAMP_HPP

namespace dsp
{
    template <typename T>
    constexpr T insertPolyBlampUpward(const T& phase, const T& increment) noexcept
    {
        const auto x = (phase - T(1)) / increment + T(1);
        return (x * x * x) * T(0.3333333333333333333);
    }
    
    template <typename T>
    constexpr T insertPolyBlampDownward(const T& phase, const T& increment) noexcept
    {
        const auto x = phase / increment - T(1);
        return (-x * x * x) * T(0.3333333333333333333);
    }
    
    //! Polynomal band limited ramp function
    template <typename T>
    constexpr T polyBlamp(const T& phase, const T& increment) noexcept
    {
        if (phase < increment)
            return insertPolyBlampDownward(phase, increment);
        else if (phase > 1 - increment)
            return insertPolyBlampUpward(phase, increment);
        else
            return T();
    }
}

#endif /* GRIZZLY_POLY_BLAMP_HPP */
