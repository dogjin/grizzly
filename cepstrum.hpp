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
    // Compute the ceptrum of a spectrum of real data
    template <class ComplexIterator>
    std::vector<typename ComplexIterator::value_type::value_type> computeRealCepstrum(FastFourierTransformBase& fft, ComplexIterator spectrumBegin, ComplexIterator spectrumEnd)
    {
        // Compute the log of the magnitudes
        std::vector<typename ComplexIterator::value_type::value_type> logs(std::distance(spectrumBegin, spectrumEnd));
        
        std::transform(spectrumBegin, spectrumEnd, logs.begin(), [](const auto& x)
                       {
                           const auto mag = std::abs(x);
                           return (mag == 0) ? std::numeric_limits<typename ComplexIterator::value_type::value_type>::lowest() : std::log(mag);
                       });
        
        // Take the inverse transform
        const std::vector<typename ComplexIterator::value_type::value_type> imaginary(logs.size(), 0);
        std::vector<typename ComplexIterator::value_type::value_type> result(fft.size, 0);
        fft.inverse(logs.data(), imaginary.data(), result.data());
        
        // Return the result
        return result;
    }
    
    //! Compute the ceptrum of real data
    template <typename T>
    std::vector<T> computeRealCepstrum(FastFourierTransformBase& fft, T* data)
    {
        // Take the Fourier transform
        const auto spectrum = fft.forward(data);
        
        // Compute the cepstrum
        return computeRealCepstrum(fft, spectrum.begin(), spectrum.end());
    }
    
    
    //! Comute the power cepstrum of a spectrum of real data
    template <class ComplexIterator>
    std::vector<typename ComplexIterator::value_type::value_type> computeRealPowerCepstrum(FastFourierTransformBase& fft, ComplexIterator spectrumBegin, ComplexIterator spectrumEnd)
    {
        // Compute the log of the magnitudes
        std::vector<typename ComplexIterator::value_type::value_type> logs(std::distance(spectrumBegin, spectrumEnd));
        std::transform(spectrumBegin, spectrumEnd, logs.begin(), [](const auto& x)
                       {
                           const auto magsq = std::norm(x);
                           return (magsq == 0) ? std::numeric_limits<typename ComplexIterator::value_type::value_type>::lowest() : std::log(magsq);
                       });
        
        // Take the inverse transform
        const std::vector<typename ComplexIterator::value_type::value_type> imaginary(logs.size(), 0);
        std::vector<typename ComplexIterator::value_type::value_type> result(fft.size, 0);
        fft.inverse(logs.data(), imaginary.data(), result.data());
        
        // Return the result, squared
        std::transform(result.begin(), result.end(), result.begin(), [](const auto& x){ return x * x; });
        return result;
    }
    
    //! Compute the power ceptrum of real data
    template <typename T>
    std::vector<T> computeRealPowerCepstrum(FastFourierTransformBase& fft, T* data)
    {
        // Take the Fourier transform
        const auto spectrum = fft.forward(data);
        
        // Compute the power cepstrum
        return computeRealPowerCepstrum(fft, spectrum.begin(), spectrum.end());
    }
    
    
    //! Compute the complex cepstrum of a spectrum of complex data
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type> computeComplexCepstrum(FastFourierTransformBase& fft, ComplexIterator spectrumBegin, ComplexIterator spectrumEnd)
    {
        auto logs = computeLogs(spectrumBegin, spectrumEnd);
        return fft.inverseComplex(logs.begin());
    }
    
    //! Compute the complex cepstrum of complex data
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type> computeComplexCepstrum(FastFourierTransformBase& fft, ComplexIterator dataBegin)
    {
        auto spectrum = fft.forwardComplex(dataBegin);
        return computeComplexCepstrum(fft, spectrum.begin(), spectrum.end());
    }
    
    //! Compute the inverse of the complex cepstrum
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type> computeInverseComplexCepstrum(FastFourierTransformBase& fft, ComplexIterator iterator)
    {
        auto spectrum = fft.forwardComplex(iterator);
        for (auto& bin : spectrum)
            bin = std::exp(bin);
        
        return fft.inverseComplex(spectrum.begin());
    }
}

#endif
