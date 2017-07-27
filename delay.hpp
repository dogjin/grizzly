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
        using iterator = typename CircularBuffer<T>::reverse_iterator;
        using const_iterator = typename CircularBuffer<T>::reverse_const_iterator;

    public:
        //! Construct by feeding the maximal delay size
        /*! @param args The arguments passed to newly constructed elements in the delay buffer */
        template <typename... Args>
        Delay(std::size_t maximalDelayTime, Args&&... args) :
            data(maximalDelayTime + 1, std::forward<Args&&>(args)...)
        {
            
        }
        
        //! Push a sample in the delay line
        template <typename... Args>
        void write(Args&&... args)
        {
            data.write(std::forward<Args&&>(args)...);
        }
        
        //! Push a new sample by existing an old one
        void adjust(std::function<void(T&)> func)
        {
            data.adjust(func);
        }
        
        //! Read from the delay line
        template <typename Index,
                  typename = std::enable_if_t<std::is_integral<Index>::value>>
        const T& read(Index index) const
        {
            return math::clampAccess(begin(), end(), index);
        }
        
        //! Read from the delay line with a fractional index
        template <typename Index, typename Interpolation>
        auto read(Index index, Interpolation interpolate) const
        {
            return interpolate(begin(), end(), index, math::clampAccess);
        }
        
        //! Set the maximal delay
        /*! @param args The arguments passed to newly constructed elements in the delay buffer */
        template <typename... Args>
        void setMaximalDelayTime(std::size_t maximalDelayTime, Args&&... args)
        {
            data.resize_front(maximalDelayTime + 1, std::forward<Args&&>(args)...);
        }
        
        //! Return the maximal number of delay samples
        std::size_t getMaximalDelayTime() const { return data.size() - 1; }
        
        // Begin and end for ranged for-loops
        iterator begin() { return data.rbegin(); }
        const_iterator begin() const { return data.rbegin(); }
        const_iterator cbegin() const { return data.crbegin(); }
        
        iterator rbegin() { return data.begin(); }
        const_iterator rbegin() const { return data.begin(); }
        const_iterator crbegin() const { return data.cbegin(); }
        
        iterator end() { return data.rend(); }
        const_iterator end() const { return data.rend(); }
        const_iterator cend() const { return data.crend(); }
        
        iterator rend() { return data.end(); }
        const_iterator rend() const { return data.end(); }
        const_iterator crend() const { return data.cend(); }
        
    private:
        //! The data in the delay line
        CircularBuffer<T> data;
    };
}

#endif
