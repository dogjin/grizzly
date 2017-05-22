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

#include <cassert>

#include "fast_fourier_transform_base.hpp"

using namespace math;
using namespace std;

namespace dsp
{
    FastFourierTransformBase::FastFourierTransformBase(size_t size) :
        size(size),
        realSpectrumSize(size / 2 + 1)
    {
        
    }
    
    Spectrum<float> FastFourierTransformBase::forward(const float* input)
    {
        Spectrum<float> output(realSpectrumSize);
        forward(input, output.begin());
        return output;
    }
    
    Spectrum<double> FastFourierTransformBase::forward(const double* input)
    {
        Spectrum<double> output(realSpectrumSize);
        forward(input, output.begin());
        return output;
    }

    vector<float> FastFourierTransformBase::inverse(const Spectrum<float>& input)
    {
        return inverse(input.begin());
    }

    vector<double> FastFourierTransformBase::inverse(const Spectrum<double>& input)
    {
        return inverse(input.begin());
    }
}
