/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2016 Dsperados <info@dsperados.com>
 
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

#ifndef GRIZZLY_COMB_FILTER_HPP
#define GRIZZLY_COMB_FILTER_HPP

#include <cstddef>
#include <functional>

#include "Delay.hpp"

namespace dsp
{
    //! Feed-back Comb Filter
    template <typename T>
    class FeedBackCombFilter
    {
    public:
        FeedBackCombFilter(const std::size_t maxDelay):
            delay(maxDelay)
        {
            
        }
        
        //! Write a new sample to the filter
        void write(const T& x, float delayTime, float feedBack)
        {
            const auto d = delay.read(delayTime, math::linearInterpolation);
            y = x + feedBack * (postDelay ? postDelay(d) : d);
            
            delay.write(y);
        }
        
        //! Return the last computed sample
        T read() const
        {
            return y;
        }
        
        //! Write a new sample to the filter and return the result
        T writeAndRead(const T& x, float delayTime, float feedBack)
        {
            write(x, delayTime, feedBack);
            return read();
        }

        void setMaxDelayTime(float maxDelayTime)
        {
            delay.resize(maxDelayTime);
        }
        
        float getMaxDelayTime()
        {
            return delay.getMaximumDelayTime();
        }
        
    public:
        //! PostDelay function
        std::function<T(const T&)> postDelay;
        
    private:
        Delay<T> delay;
        
        //! The computed y output value
        T y;
    };
    
    //! Feed-forward Comb Filter
    template <class T>
    class FeedForwardCombFilter
    {
    public:
        FeedForwardCombFilter(const std::size_t maxDelay):
        	delay(maxDelay)
        {
            
        }
        
        //! Write a new sample to the filter
        void write(const T& x, float delayTime, float feedForward)
        {
            delay.write(x);
            
            const auto d = delay.read(delayTime, math::linearInterpolation);
            y = x + feedForward * (postDelay ? postDelay(d) : d);
        }
        
        //! Return the last computed sample
        T read() const
        {
            return y;
        }
        
        //! Write a new sample to the filter and return the result
        T writeAndRead(const T& x, float delayTime, float feedBack)
        {
            write(x, delayTime, feedBack);
            return read();
        }
        
        void setMaxDelayTime(float maxDelayTime)
        {
            delay.resize(maxDelayTime);
        }
        
        float getMaxDelayTime()
        {
            return delay.getMaximumDelayTime ();
        }
        
    public:
        //! PostDelay function
        std::function<T(const T&)> postDelay;
        
    private:
        Delay<T> delay;
        
        //! The computed y output value
        T y = 0;
    };
}
#endif /* GRIZZLY_COMB_FILTER_HPP */
