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

#ifndef GRIZZLY_CEPSTRUM_HPP
#define GRIZZLY_CEPSTRUM_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include "fast_fourier_transform.hpp"

namespace dsp
{
    std::vector<float> powerCepstrum(FastFourierTransformBase& fft, float* data)
    {
        // Take the Fourier transform
        auto spectrum = fft.forward(data);
        
        // Compute the squared magnitudes
        std::vector<float> mags(spectrum.size());
        std::transform(spectrum.real.begin(), spectrum.real.end(),
                       spectrum.imaginary.begin(),
                       mags.begin(), [](const auto& r, const auto& i){ return r * r + i * i; });
        
        std::vector<float> result(fft.size);
        
//        // Take the log of the squares of the magnitudes
//        auto magnitudes = spectrum.magnitudes();
//        std::transform(magnitudes.begin(), magnitudes.end(), magnitudes.begin(), [](const auto& x){ return std::log(x * x); });
//        
//        // Do the inverse transform
//        std::vector<float> imaginary(magnitudes.size(), 0);
//        std::vector<float> result(fft.size);
//        fft.inverse(magnitudes.data(), imaginary.data(), result.data());
//        
//        // Square the result
//        std::transform(result.begin(), result.end(), result.begin(), [](const auto& x){ return x * x; });
        
        return result;
    }
}

#endif
