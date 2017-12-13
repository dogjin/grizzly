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

#ifndef GRIZZLY_NORMALIZED_SATURATOR_HPP
#define GRIZZLY_NORMALIZED_SATURATOR_HPP

#include <cmath>
#include <functional>

namespace dsp
{
    template <class T, class Function>
    class NormalizedSaturator
    {
    public:
        NormalizedSaturator(Function function) :
            saturator(function)
        {
            setDownwardFactor(1);
            setUpwardFactor(1);
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
        
        void setDownAndUpwardFactor(float factorDown, float factorUp)
        {
            setDownwardFactor(factorDown);
            setUpwardFactor(factorUp);
        }
        
        // set both factor (iets sneller)
        void setFactor(float factor)
        {
            upwardFactor = factor;
            downwardFactor = factor;
            
            upwardNormalizeFactor = 1.0 / saturator(upwardFactor);
            downwardNormalizeFactor = upwardNormalizeFactor;
        }
        
    private:
        const Function saturator;
        
        float upwardFactor = 1;
        float upwardNormalizeFactor = 1;
        
        float downwardFactor = 1;
        float downwardNormalizeFactor = 1;
    };
    
    template <class T>
    using NormalizedSaturatorFunctionPtr = NormalizedSaturator<T, T(*)(T)>;
    
    template <class T>
    using NormalizedSaturatorStdFunction = NormalizedSaturator<T, std::function<T(T)>>;
    
    template <class T>
    static NormalizedSaturatorFunctionPtr<T> NORMALIZED_TANH(std::tanh);
    
    template <class T>
    static NormalizedSaturatorFunctionPtr<T> NORMALIZED_ATAN(std::atan);
}

#endif /* GRIZZLY_NORMALIZED_SATURATOR_HPP */
