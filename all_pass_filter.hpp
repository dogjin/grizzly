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

#ifndef GRIZZLY_ALL_PASS_FILTER_HPP
#define GRIZZLY_ALL_PASS_FILTER_HPP

#include <dsperados/math/constants.hpp>
#include <stdexcept>
#include <vector>

#include "delay.hpp"

using namespace std;

namespace dsp
{
    //! An n-th order all pass filter
    /*! Alter the phase response of a signal leaving the magnitudes unaltered.
     The delay (order) can be fractional and different interpolators can be chosen.
     One delay-line is used in a transposed direct form II fashion. */
    template <class T>
    class AllPassFilter
    {
    public:
        //! Construct the all-pass filter
        AllPassFilter(std::size_t maximalDelayTime) :
        delay(maximalDelayTime)
        {
            
        }
        
        //! Write a new sample to the filter given an a0 (gain) coefficient and default linear interpolation
        /*! For reverberators, see math::INVERSE_PHI as a possible a0 coefficient. */
        template <typename Interpolation = decltype(math::linearInterpolation)>
        void write(const T& x, float delayTime, float a0, Interpolation interpolation = math::linearInterpolation)
        {
            if (delayTime < 1)
                throw std::invalid_argument("delay time < 1");
            
            // read delay with time minus 1 because the previous call did the write, which already introduced a delay
            auto z1 = delay.read(delayTime - 1, interpolation);
            
            // write output
            y = x * a0 + z1;
            
            // Update the delay-line
            delay.write(x + y * -a0);
        }
        
        //! Return the most recently computed output
        T read() const { return y; }
        
        //! Write a new sample and read the result (in that order)
        template <typename Interpolation = decltype(math::linearInterpolation)>
        T writeAndRead(const T& x, float delayTime, float a0, Interpolation interpolation = math::linearInterpolation)
        {
            write(x, delayTime, a0, interpolation);
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
