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

#ifndef GRIZZLY_ROUNDED_CLIP_HPP
#define GRIZZLY_ROUNDED_CLIP_HPP

namespace dsp
{
    template <class T>
    class RoundedClip
    {
    public:
        RoundedClip() = default;
        
        RoundedClip(float threshold) :
            threshold(threshold)
        {
            
        }
        
        T process(T x)
        {
            if (std::abs(x) < threshold)
                return x;
            
            const float oneMinusThrehold = 1.f - threshold;
            
            if (x > 0)
            {
                const auto interpolatedValue = computeInterpolatedValue((x - threshold) / (oneMinusThrehold * 1.5));
                return oneMinusThrehold * interpolatedValue + threshold;
            }
            else if (x < 0)
            {
                const auto interpolatedValue = computeInterpolatedValue((-x - threshold) / (oneMinusThrehold * 1.5));
                return oneMinusThrehold * -interpolatedValue - threshold;
            }
            
            return 0;
        }
        
        T operator()(T x)
        {
            return process(x);
        }
        
        void setThreshold(float threshold)
        {
            this->threshold = threshold;
        }
        
    private:
        T computeInterpolatedValue(T x)
        {
            if (std::abs(x) < 1)
                return x * (1.5 - 0.5 * x * x);
            else
                return x > 0 ? 1 : -1;
        }
        
    private:
        float threshold = 1;
    };
}

#endif /* GRIZZLY_ROUNDED_CLIP_HPP */
