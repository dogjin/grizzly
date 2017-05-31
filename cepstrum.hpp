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

#include <cmath>
#include <complex>
#include <limits>
#include <vector>

#include "complex.hpp"
#include "fast_fourier_transform_base.hpp"

namespace dsp
{
    template <typename T>
    std::vector<T> cepstrum(FastFourierTransformBase& fft, T* data)
    {
        // Take the Fourier transform
        const auto spectrum = fft.forward(data);
        
        // Compute the log of the magnitudes
        std::vector<T> logs(spectrum.size());
        std::transform(spectrum.begin(), spectrum.end(), logs.begin(), [](const auto& x) -> T
        {
            const auto mag = std::abs(x);
            return (mag == 0) ? std::numeric_limits<T>::lowest() : std::log(mag);
        });
        
        // Take the inverse transform
        const std::vector<T> imaginary(logs.size(), 0);
        std::vector<T> result(fft.size, 0);
        fft.inverse(logs.data(), imaginary.data(), result.data());
        
        // Return the result
        return result;
    }
    
    template <typename T>
    std::vector<float> powerCepstrum(FastFourierTransformBase& fft, T* data)
    {
        // Take the Fourier transform
        const auto spectrum = fft.forward(data);
        
        // Compute the log of the magnitudes
        std::vector<T> logs(spectrum.size());
        std::transform(spectrum.begin(), spectrum.end(), logs.begin(), [](const auto& x) -> T
        {
            const auto magsq = std::norm(x);
            return (magsq == 0) ? std::numeric_limits<T>::lowest() : std::log(magsq);
        });
        
        // Take the inverse transform
        const std::vector<T> imaginary(logs.size(), 0);
        std::vector<T> result(fft.size, 0);
        fft.inverse(logs.data(), imaginary.data(), result.data());
        
        // Return the result, squared
        std::transform(result.begin(), result.end(), result.begin(), [](const auto& x){ return x * x; });
        return result;
    }
    
    //! Take the complex cepstrum
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type> cepstrumComplex(FastFourierTransformBase& fft, ComplexIterator iterator)
    {
        auto spectrum = fft.forwardComplex(iterator);
        log(spectrum.begin(), spectrum.end(), spectrum.begin());
        return fft.inverseComplex(spectrum.begin());
    }
    
    //! Take the inverse of the complex cepstrum
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type> cepstrumComplexInverse(FastFourierTransformBase& fft, ComplexIterator iterator)
    {
        auto spectrum = fft.forwardComplex(iterator);
        for (auto& bin : spectrum)
            bin = std::exp(bin);
        
        return fft.inverseComplex(spectrum.begin());
    }
}

#endif
