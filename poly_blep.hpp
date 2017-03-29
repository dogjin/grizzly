/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2017 Dsperados <info@dsperados.com>
 
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

#ifndef POLY_BLEP_HPP
#define POLY_BLEP_HPP

namespace dsp
{
    //! Polinomal band limited step function
    static inline double polyBlep(double phase, double increment)
    {
        if (phase < increment)
        {
            double x = phase / increment;
            return x + x - x * x - 1.0;
        }
        else if (phase > 1.0 - increment)
        {
            double x = (phase - 1.0) / increment;
            return x * x + x + x + 1.0; // x*x * (3 - x * x) // x*x*x * (x * (x * 6 - 15) + 10)
        }
        else return 0;
    }
}

#endif
