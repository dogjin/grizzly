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

#ifndef GRIZZLY_ALL_PASS_FILTER_HPP
#define GRIZZLY_ALL_PASS_FILTER_HPP

#include <dsperados/math/constants.hpp>
#include <vector>

#include "Delay.hpp"

using namespace std;

namespace dsp
{
    //! An all pass filter
    template <class T>
    class AllPassFilter
    {
    public:
        //! Construct the all-pass filter
        AllPassFilter(std::size_t maximalDelayTime) :
            delay(maximalDelayTime)
        {

        }

        //! Write a new sample to the filter
        void write(const T& x, float delayTime, float gain = math::INVERSE_PHI<float>)
        {
            const auto d = delay.read(delayTime, math::linearInterpolation);
            const auto w = gain * d + x;
            y = gain * w - d;

            delay.write(w);
        }
        
        //! Return the most recently computed output
        T read() const { return y; }
        
        //! Write a new sample and read the result
        T writeAndRead(const T& x, float delayTime, float gain = math::INVERSE_PHI<float>)
        {
            write(x, delayTime, gain);
            return read();
        }
        
        //! Change the maximal delay time
        void setMaximalDelayTime(std::size_t maximalDelayTime)
        {
            delay.setMaximalDelayTime(maximalDelayTime);
        }

        //! Return the maximal delay time that can be used
        std::size_t getMaximalDelayTime() const
        {
            return delay.getMaximalDelayTime();
        }

    private:
        //! The delay line
        Delay<T> delay;
        
        //! The output value
        T y;
    };
}


#endif
