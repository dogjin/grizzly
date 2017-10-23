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
 along with this program. If not, see <http://www.gnu.org/licenses/>
 
 --------------------------------------------------------------------
 
 If you would like to use Grizzly for commercial or closed-source
 purposes, please contact us for a commercial license.
 
 */

#ifndef GRIZZLY_POLY_BLEP_HPP
#define GRIZZLY_POLY_BLEP_HPP

namespace dsp
{
    template <typename T>
    inline T insertPolyBlepAfterReset(const T& phase, const T& increment)
    {
        const auto x = phase / increment;
        return x + x - x * x - T(1);
    }
    
    template <typename T>
    inline T insertPolyBlepBeforeReset(const T& phase, const T& increment)
    {
        const auto x = (phase - 1.0) / increment;
        return x * x + x + x + T(1);
    }
    
    //! Polynomal band limited step function
    template <typename T>
    inline T polyBlep(const T& phase, const T& increment)
    {
        if (phase < increment)
            return insertPolyBlepAfterReset(phase, increment);
        else if (phase > 1.0 - increment)
            return insertPolyBlepBeforeReset(phase, increment);
        else
            return T();
    }
}

#endif
