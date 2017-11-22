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

#ifndef GRIZZLY_HARD_CLIP_HPP
#define GRIZZLY_HARD_CLIP_HPP

#include <cmath>

namespace dsp
{
    template <typename T>
    T hardClip(const T& x, const T& threshold)
    {
        if (std::abs(x) > threshold)
            return std::signbit(x) ? -threshold : threshold;
        else
            return x;
    }
    
    template <class T>
    class HardClipAntiAliased
    {
    public:
        T process(const T& x, const T& threshold)
        {
            T y = 0;
            
            // Implement clipping and detect clipping points (corners)
            if (std::abs(x) >= threshold)
            {
                clipState = true;
                y = signbit(x) ? -threshold : threshold;
            }
            else
            {
                clipState = false;
                y = x;
            }
            
            // If clipping point then implement correction
            if (clipState != previousClipState)
            {
                float direction = signbit(xz1) ? -1 : 1;
                float m = x - xz1;
                float d = (direction * threshold - xz1) / m;
                
                float p1 = -(d*d*d)/6.0 + (d*d)/2.0 - d/2.0 + 1.0/6.0;
                float p0 = (d*d*d)/6.0;
                
                yz1 -= direction * std::abs(m) * p1;
                y -= direction * std::abs(m) * p0;
            }
            
            T out = yz1;
            
            yz1 = y;
            xz1 = x;
            previousClipState = clipState;
            
            return out;
        }
        
    private:
        T yz1 = 0;
        T xz1 = 0;
        
        bool clipState = false;
        bool previousClipState = false;
    };
}


#endif
