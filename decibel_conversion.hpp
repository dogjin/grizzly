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

#pragma once

#include <cmath>
#include <limits>
#include <numeric>

namespace dsp
{
    /*! @brief Convert linear amplitude to deciBel
     *
     */
    template <typename T>
    T amplitudeToDecibel(T value)
    {
        // get the smallest possible positive value for T
        const T epsilon = std::numeric_limits<T>::min();
        
        // if the value is lower, use the epsilon
        if (value < epsilon)
            value = epsilon;
        
        return 20 * std::log10(value);
    }
    
    /*! @brief Convert deciBel to linear amplitude
     *
     */
    template <typename T>
    T decibelToAmplitude(T value)
    {
        const auto y = std::pow(T(10), value * T(0.05));
        if (std::isinf(y))
            return std::numeric_limits<T>::max();
        else
            return y;
    }
}
