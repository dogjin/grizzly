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


#pragma once

#include <cmath>
#include <utility>

#include <moditone/math/ease.hpp>

#include "mid_side.hpp"

namespace dsp
{
    template <typename T>
    Stereo<T> panLinear(float direction)
    {
        auto l = math::scale<float>(direction, -1.f, 1.f, 1.f, 0.f);
        auto r = math::scale<float>(direction, -1.f, 1.f, 0.f, 1.f);
        
        return {l, r};
    }
    
    template <typename T>
    Stereo<T> panCosine(float direction)
    {
        auto l = math::scale<float>(direction, -1.f, 1.f, 1.f, 0.f);
        auto r = math::scale<float>(direction, -1.f, 1.f, 0.f, 1.f);
        
        return {math::easeSineOut(l), math::easeSineOut(r)};
    }
    
    template <typename T>
    Stereo<T> panExponential(float direction, float exponent)
    {
        auto l = math::scale<float>(direction, -1.f, 1.f, 1.f, 0.f);
        auto r = math::scale<float>(direction, -1.f, 1.f, 0.f, 1.f);
        
        return {std::pow(l, exponent), std::pow(r, exponent)};
    }
}





//template <typename T>
//std::pair<T,T> panLinearStereo(const T& direction)
//{
//    return std::make_pair(0.5 * (1 - direction), 0.5 * (1 + direction));
//}
//
//template <typename T>
//std::pair<T,T> panEqualPowerStereo(const T& direction)
//{
//    return std::make_pair(std::sqrt(0.5 * (1 - direction)), std::sqrt(0.5 * (1 + direction)));
//}
//
//template <typename T>
//std::pair<T,T> panCosineStereo(const T& direction)
//{
//    auto x = (direction + 1) / 4 * math::PI<T>;
//    return std::make_pair(std::cos(x), std::sin(x));
//}
