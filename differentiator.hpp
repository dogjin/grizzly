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

#pragma once

namespace dsp
{
    /*! @brief Differentiator
     *
     *  @discussion Compute the difference of
     *  of an input value with its lastly
     *  computed state.
     *
     */
    template <class T>
    class Differentiator
    {
    public:
        //! Differentiate given an input vlaue
        T process(T x)
        {
            // compute difference
            const auto y = x - previousX;
            
            // update previous input
            previousX = x;
            
            return y;
        }
        
        //! Differentiate given an input vlaue
        T operator()(const T& x)
        {
            return process(x);
        }
        
    public:
        //! The previous value, state
        T previousX = 0;
    };
}