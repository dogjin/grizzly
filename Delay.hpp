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

#ifndef GRIZZLY_DELAY_HPP
#define GRIZZLY_DELAY_HPP

#include <dsperados/math/interpolation.hpp>

#include "circular_buffer.hpp"

namespace dsp
{
    //! A simple sample delay object
    /*! Delay based on a circular buffer, capable of interpolation */
    template <typename T>
    class Delay
    {
    public:
        //! Construct by feeding the maximal delay size
        Delay(std::size_t maximalDelayTime) :
            data(maximalDelayTime + 1)
        {
            
        }
        
        //! Push a sample in the delay line
        template <typename... Args>
        void write(Args&&... args)
        {
            data.emplace_back(std::forward<Args&&>(args)...);
        }
        
        //! Read from the delay line
        template <typename Index>
        std::enable_if_t<std::is_integral<Index>::value, T> read(Index index) const
        {
            return access(data.rbegin(), data.rend(), index, math::clampAccess);
        }
        
        //! Read from the delay line with a fractional index
        template <typename Index, typename Interpolator>
        T read(Index index, Interpolator interpolator) const
        {
            return interpolate(data.rbegin(), data.rend(), index, interpolator, math::clampAccess);
        }
        
        //! Set the maximal delay
        void setMaximalDelayTime(std::size_t maximalDelayTime)
        {
            data.resize_front(maximalDelayTime + 1);
        }
        
        //! Return the maximal number of delay samples
        std::size_t getMaximalDelayTime() const { return data.size() - 1; }
        
    private:
        //! The data in the delay line
        CircularBuffer<T> data;
    };
}

#endif
