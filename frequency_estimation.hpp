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

#ifndef GRIZZLY_FREQUENCY_ESTIMATION_HPP
#define GRIZZLY_FREQUENCY_ESTIMATION_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <dsperados/math/analysis.hpp>
#include <dsperados/math/interpolation.hpp>
#include <experimental/optional>
#include <iterator>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include <utility>

#include "cepstrum.hpp"

namespace dsp
{
    //! Compute the Normalized Square Difference of a signal
    /*! Autocorrelation function (ACF) is defined as a sum of x[i] * x[i + tau].
        Square Difference function (SDF) is defined as a sum of (x[i] - x[i + tau])^2.
        Expanding SDF yields a sum of (x[i]^2 + x[i + tau]^2 - 2x[i] * x[i + tau]).
        So there is a ACF in the SDF.
        Let m be the sum of (x[i]^2 + x[i + tau]^2).
        let r be the ACF part.
        Then SDF is defiend as m - 2r.
        Normalising the SDF (NSDF) results in +2r / m.
        The sum runs over each window but decreases with increasing the tau (index + windowSize - 1 - tau). */
    template <class Iterator>
    static inline std::vector<float> computeNormalizedSquareDifference(Iterator begin, Iterator end)
    {
        auto size = std::distance(begin, end);
        // auto size = frame.size();
        std::vector<float> squareDifference(size);
        std::vector<float> autoCorrelation(size);
        
        // The normalizedSDF output
        std::vector<float> normalizedSquareDifference;
        normalizedSquareDifference.reserve(size);
        
        // for every tau (lag) compute the SDF and ACF
        for (auto tau = 0; tau < size; tau++)
        {
            auto it = begin;
            auto tauIt = std::next(begin, tau);
            // for every index in window size decreasing with each tau
            for (auto i = 0; i < (size - tau); i++)
            {
                squareDifference[tau] += *it * *it + *tauIt * *tauIt;
                autoCorrelation[tau] += *it * *tauIt;
                
                std::advance(it, 1);
                std::advance(tauIt, 1);
            }
            
            normalizedSquareDifference.emplace_back((2 * autoCorrelation[tau]) / squareDifference[tau]);
        }
        
        return normalizedSquareDifference;
    }
    
    //! Estimate the frequency given the result of a Normalized Square Difference, samplerate and a tolerace (usually in the range 0.8 to 1.0)
    /*! See "A Smarter Way to Find Pitch" by Philip McLeod and Geoff Wyvill.
        http://miracle.otago.ac.nz/tartini/papers/A_Smarter_Way_to_Find_Pitch.pdf */
    static inline float estimateFrequencyWithNormalizedSquareDifference(const std::vector<float>& normalizedSquareDifference, float sampleRate, float tolerance)
    {
        // Find all peaks in the result of the normalized square difference functions
        auto localMaximaPositions = math::findLocalMaximaPositions(normalizedSquareDifference.begin(), normalizedSquareDifference.end());
        
        // Return a frequency of zero if no peaks are found
        if (localMaximaPositions.empty())
            return 0;
        
        // Compute all the interploated peak positions and values
        std::vector<std::pair<float, float>> interpolatedPeakPositionsAndValues;
        interpolatedPeakPositionsAndValues.reserve(localMaximaPositions.size());
        for (auto position : localMaximaPositions)
        {
            auto peakOffsetAndValue = math::interpolateParabolic(normalizedSquareDifference[position - 1], normalizedSquareDifference[position], normalizedSquareDifference[position + 1]);
            interpolatedPeakPositionsAndValues.emplace_back(position + peakOffsetAndValue.first, peakOffsetAndValue.second);
        }
        
        // Search through the interpolated peaks and return the maximum value
        auto threshold = tolerance * std::max_element(interpolatedPeakPositionsAndValues.begin(), interpolatedPeakPositionsAndValues.end(), [&](const auto& a, const auto& b){ return a.second < b.second; })->second;
        
        // Find the first peak value exceeding the threshold, return the frequency as the sampling rate divided by the peak position
        for (auto peak : interpolatedPeakPositionsAndValues)
            if (peak.second > threshold)
                return sampleRate / peak.first;
        
        throw std::runtime_error("Should never reach here, there must be a peak value > threshold");
    }
    
    //! Estimate the frequency of a signal using the normalized square difference given a samplerate and a tolerace (usually in the range 0.8 to 1.0)
    /*! See "A Smarter Way to Find Pitch" by Philip McLeod and Geoff Wyvill.
        http://miracle.otago.ac.nz/tartini/papers/A_Smarter_Way_to_Find_Pitch.pdf */
    template <typename Iterator>
    float estimateFrequencyWithNormalizedSquareDifference(Iterator begin, Iterator end, float sampleRate, float tolerance)
    {
        auto normalizedSquareDifference = computeNormalizedSquareDifference(begin, end);
        return estimateFrequencyWithNormalizedSquareDifference(normalizedSquareDifference, sampleRate, tolerance);
    }
    
    //! Estimate the frequency of a signal using the Yin algorithm given a samplerate and threshold (usually around 0.1)
    /*! See "YIN, a fundamental frequency estimator for speech and music" by Alain de Cheveigne and Hideki Kawahara.
        http://audition.ens.fr/adc/pdf/2002_JASA_YIN.pdf */
    template <typename Iterator>
    std::pair<float, float> computeYin(Iterator begin, Iterator end, float sampleRate, float threshold)
    {
        const std::size_t size = std::distance(begin, end);
        if (size < 2)
            return {0, 0};
        
        const std::size_t halfSize = size / 2;
        
        // 1. Subtract the slided version of the input from the input itself
        // 2. Raise these differences to the 2nd power
        // 3. Accumulate these powers and store them in the buffer for each slide delay
        std::vector<float> slides(halfSize);
        slides[0] = 1;
        float sum = 0;
        std::experimental::optional<unsigned int> minIndex;
        for (auto slideIndex = 1; slideIndex < halfSize; ++slideIndex)
        {
            auto& slide = slides[slideIndex];
            
            // Loop over each sample in the buffer, and subtract the slide from it
            for (auto i = 0; i < halfSize; ++i)
            {
                const auto x = begin[i] - begin[i + slideIndex];
                slide += x * x;
            }
            
            // Normalize the slide by computing the cumulative mean with the first slide set to one
            sum += slide;
            slide *= slideIndex / sum;
            
            // If we've reached a minimum, we can stop computing new slides
            if (minIndex && slides[*minIndex] < slide)
                break;
            
            // If we've dropped below the threshold, and not reached a minimum yet, store this slide
            // as the current minimum
            if (slide < threshold)
                minIndex = slideIndex;
        }
        
        // No pitch found
        if (!minIndex)
            return {0, 0};
        
        // We found the minimum, now compute the actual minimum using parabolic interpolation
        const auto& minValue = slides[*minIndex];
        
        // Apply parabolic interpolation
        const auto leftBound = math::clamp<std::size_t>(*minIndex - 1, 0, halfSize - 1);
        const auto rightBound = math::clamp<std::size_t>(*minIndex + 1, 0, halfSize - 1);
        
        // Compute the probability
        const auto probability = 1 - minValue;
        
        // Return the pitch and its probability
        if (leftBound == *minIndex)
            return {sampleRate / (minValue <= slides[rightBound] ? *minIndex : rightBound), probability};
        else if (rightBound == *minIndex)
            return {sampleRate / (minValue <= slides[leftBound] ? *minIndex : leftBound), probability};
        else
            // Parabolically interpolate to get a better local minimum, use the offset of the peak as correction on the index
            return {sampleRate / (*minIndex + math::interpolateParabolic(slides[leftBound], minValue, slides[rightBound]).first), probability};
    }
    
    //! Estimate the frequency of a buffer using Cepstrum analysis
    /*! @param lowTimeRemoval The amount of low-time quefrencies to leave out of the analysis. Usually around 0.025 - 0.1. */
    template <typename T>
    float estimateFrequencyWithCepstrum(FastFourierTransformBase& fft, T* data, float sampleRate, float lowTimeRemoval)
    {
        const auto halfSize = fft.size * 0.5;
        
        const auto cs = computeCepstrum(fft, data);
        const auto peak = std::max_element(cs.begin() + halfSize * lowTimeRemoval, cs.begin() + halfSize, std::less<>());
        return sampleRate / std::distance(cs.begin(), peak);
    }
}

#endif
