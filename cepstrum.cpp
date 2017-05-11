//
//  cepstrum.cpp
//  Grizzly
//
//  Created by Stijn on 11/05/2017.
//
//

#include <algorithm>
#include <cmath>

#include "cepstrum.hpp"
#include "fast_fourier_transform_base.hpp"

namespace dsp
{
    std::vector<float> cepstrum(FastFourierTransformBase& fft, float* data)
    {
        // Take the Fourier transform
        const auto spectrum = fft.forward(data);
        
        // Compute the log of the magnitudes
        std::vector<float> real(spectrum.size());
        std::transform(spectrum.begin(), spectrum.end(), real.begin(), [](const auto& bin){ return std::log(std::abs(bin)); });
        
        // Take the inverse transform
        const std::vector<float> imaginary(real.size(), 0);
        std::vector<float> result(fft.size, 0);
        fft.inverse(real.data(), imaginary.data(), result.data());
        
        // Return the result
        return result;
    }
    
    std::vector<float> powerCepstrum(FastFourierTransformBase& fft, float* data)
    {
        // Take the Fourier transform
        const auto spectrum = fft.forward(data);
        
        // Compute the log squared magnitudes
        std::vector<float> real(spectrum.size());
        std::transform(spectrum.begin(), spectrum.end(), real.begin(), [](const auto& bin){ return std::log(std::norm(bin)); });
        
        // Take the inverse transform
        const std::vector<float> imaginary(real.size(), 0);
        std::vector<float> result(fft.size);
        fft.inverse(real.data(), imaginary.data(), result.data());
        
        // Return the squared the result
        std::transform(result.begin(), result.end(), result.begin(), [](const auto& x){ return x * x; });
        return result;
    }
}
