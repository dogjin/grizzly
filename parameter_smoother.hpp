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
        ParameterSmoother(unit::second<double> time, unit::hertz<double> sampleRate) :
        filter(sampleRate)
        {
            filter.setTime(time, 5);
        }
        
        void write()
        {
            if (reachedDestination())
                return;
            
            if (destination > value)
            {
                value = filter.writeAndReadLowPass(destination + offset);
                
                // correct for overshoot
                if (value > destination)
                    value = destination;
            }
            else
            {
                value = filter.writeAndReadLowPass(destination - offset);
                
                // correct for undershoot
                if (value < destination)
                    value = destination;
            }
        }
        
        T read()
        {
            return value;
        }
        
        T writeAndRead()
        {
            write();
            return read();
        }
        
        void setTime(unit::second<double> time)
        {
            filter.setTime(time);
        }
        
        void setSampleRate(unit::hertz<float> sampleRate)
        {
            filter.setSampleRate(sampleRate.value);
        }
        
        bool reachedDestination()
        {
            return (value == destination) ? true : false;
        }
        
    public:
        T destination = 0;
        T value = 0;
        T offset = 0.0000001;
        
    private:
        StateVariableFilter<float> filter;
        
    };
}


#endif /* GRIZZLY_PARAMETER_SMOOTHER_HPP */
