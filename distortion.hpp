//
//  distortion.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 13/06/2017.
//
//

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
