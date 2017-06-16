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

#ifndef GRIZZLY_DISTORTION_HPP
#define GRIZZLY_DISTORTION_HPP

#include <cmath>

#include "clip.hpp"

namespace dsp
{
    template <class T, T (*saturator)(T)>
    class normalizedSaturator
    {
    public:
        normalizedSaturator()
        {
            setDownwardFactor(1);
            setUpwardFactor(1);
        }
        
        normalizedSaturator(float downwardFactor, float upwardFactor)
        {
            if (downwardFactor <= 0 || upwardFactor <=0)
                throw std::runtime_error("downwardFactor <= 0 || upwardFactor <= 0");
            
            setDownwardFactor(downwardFactor);
            setUpwardFactor(upwardFactor);
        }
        
        T process(const T& x)
        {
            if (x > 0)
                return saturator(x * upwardFactor) * upwardNormalizeFactor;
            
            if (x < 0)
                return saturator(x * downwardFactor) * downwardNormalizeFactor;
            
            return 0;
        }
        
        T operator()(const T& x)
        {
            return process(x);
        }
        
        void setUpwardFactor(float factor)
        {
            upwardFactor = factor;
            upwardNormalizeFactor = 1.0 / saturator(upwardFactor);
        }
        
        void setDownwardFactor(float factor)
        {
            downwardFactor = factor;
            downwardNormalizeFactor = 1.0 / saturator(downwardFactor);
        }
        
    private:
        float upwardFactor = 1;
        float upwardNormalizeFactor = 1;
        
        float downwardFactor = 1;
        float downwardNormalizeFactor = 1;
    };
}

/*
 namespace dsp
 {
 template <class T>
 class normalizedSaturator
 {
 public:
 normalizedSaturator()
 {
 setDownwardFactor(1);
 setUpwardFactor(1);
 }
 
 normalizedSaturator(float downwardFactor, float upwardFactor)
 {
 if (downwardFactor <= 0 || upwardFactor <=0)
 throw std::runtime_error("downwardFactor <= 0 || upwardFactor <= 0");
 
 setDownwardFactor(downwardFactor);
 setUpwardFactor(upwardFactor);
 }
 
 T process(const T& x)
 {
 if (x > 0)
 return saturator(x * upwardFactor) * upwardNormalizeFactor;
 
 if (x < 0)
 return saturator(x * downwardFactor) * downwardNormalizeFactor;
 
 return 0;
 }
 
 void setUpwardFactor(float factor)
 {
 upwardFactor = factor;
 upwardNormalizeFactor = 1.0 / saturator(upwardFactor);
 }
 
 void setDownwardFactor(float factor)
 {
 downwardFactor = factor;
 downwardNormalizeFactor = 1.0 / saturator(downwardFactor);
 }
 
 void setSaturator(std::function<T(T)> saturator)
 {
 this->saturator = saturator;
 
 setDownwardFactor(downwardFactor);
 setUpwardFactor(upwardFactor);
 }
 
 std::function<T(T)> atanSaturator = [](const T& x){ return std::atan(x); };
 std::function<T(T)> tanhSaturator = [](const T& x){ return std::tanh(x); };
 
 private:
 std::function<T(T)> saturator = atanSaturator;
 
 float upwardFactor = 1;
 float upwardNormalizeFactor = 1;
 
 float downwardFactor = 1;
 float downwardNormalizeFactor = 1;
 };
 }
 */

#endif
