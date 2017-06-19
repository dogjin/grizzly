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

#ifndef GRIZZLY_PARAMETER_SMOOTHER_HPP
#define GRIZZLY_PARAMETER_SMOOTHER_HPP

#include "first_order_filter.hpp"
#include "state_variable_filter.hpp"

namespace  dsp
{
    template <class T, class CoeffType = T>
    class ParameterSmoother
    {
    public:
        ParameterSmoother(unit::second<float> time, unit::hertz<float> sampleRate) :
            filter(time, sampleRate)
        {
        }
        
        void write()
        {
            filter.write(destination);
        }
        
        T read()
        {
            return filter.readLowPass();
        }
        
        T writeAndRead()
        {
            write();
            return read();
        }
        
        void setTime(unit::second<float> time)
        {
            filter.setTime(time);
        }
        
        void setSampleRate(unit::hertz<float> sampleRate)
        {
            filter.setSampleRate(sampleRate);
        }
        
        bool reachedDestination()
        {
            return (filter.readLowPass() == destination) ? true : false;
        }
        
    public:
        T destination = 0;
        
    private:
        StateVariableFilter<float> filter;

    };
}


#endif /* GRIZZLY_PARAMETER_SMOOTHER_HPP */
