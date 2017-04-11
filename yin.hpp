/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2016 Dsperados <info@dsperados.com>
 
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

#ifndef GRIZZLY_YIN_HPP
#define GRIZZLY_YIN_HPP

#include <cstddef>
#include <cmath>
#include <iterator>
#include <utility>
#include <vector>

#include <dsperados/math/interpolation.hpp>

namespace dsp
{
    //! Yin pitch tracking algorithm
    /*! Compute the pitch over a range of samples give a sampling rate and a threshold.
        Returns the pitch and probability in a std::pair.
        See "YIN, a fundamental frequency estimator for speech and music" by Alain de Cheveigne and Hideki Kawahara.
        http://audition.ens.fr/adc/pdf/2002_JASA_YIN.pdf */
    template <typename Iterator>
    std::pair<float, float> computeYin(Iterator begin, Iterator end, float sampleRate, float threshold = 0.15)
    {
        const std::size_t size = std::distance(begin, end);
        const std::size_t halfSize = size / 2;
        
        // 1. Subtract the slided version of the input from the input itself
        // 2. Raise these differences to the 2nd power
        // 3. Accumulate these powers and store them in the buffer
        std::vector<float> slides(halfSize);
        for (auto slide = 0 ; slide < halfSize; ++slide)
        {
            // Loop over each sample in the buffer, and subtract the slide from it
            for (auto i = 0; i < halfSize; ++i)
            {
                const auto x = begin[i] - begin[i + slide];
                slides[slide] += x * x;
            }
        }
        
        // Normalize the slides, by computing the cumulative mean with the first slide set to one
        slides[0] = 1;
        float sum = 0;
        for (int slide = 1; slide < halfSize; ++slide)
        {
            sum += slides[slide];
            slides[slide] *= slide / sum;
        }
        
        // Search for the first slide below the threshold
        std::size_t minIndex = 2;
        for (minIndex = 2; minIndex < halfSize; ++minIndex)
        {
            if (slides[minIndex] < threshold)
                break;
        }
        
        // Descend to local minimum
        while (minIndex + 1 < halfSize && slides[minIndex + 1] < slides[minIndex])
            minIndex++;
        
        const auto& minValue = slides[minIndex];
        
        // No pitch found
        if (minIndex == halfSize || minValue >= threshold)
            return {0, 0};
        
        // Apply parabolic interpolation
        const auto leftBound = math::clamp<std::size_t>(minIndex - 1, 0, halfSize - 1);
        const auto rightBound = math::clamp<std::size_t>(minIndex + 1, 0, halfSize - 1);
        
        // Compute the probability
        const auto probability = 1 - minValue;
        
        // Return the pitch and its probability
        if (leftBound == minIndex)
            return {sampleRate / (minValue <= slides[rightBound] ? minIndex : rightBound), probability};
        else if (rightBound == minIndex)
            return {sampleRate / (minValue <= slides[leftBound] ? minIndex : leftBound), probability};
        else
            // Parabolically interpolate to get a better local minimum, use the offset of the peak as correction on the index
            return {sampleRate / (minIndex + math::interpolateParabolic(slides[leftBound], minValue, slides[rightBound]).first), probability};
    }
}

#endif /* GRIZZLY_YIN_HPP */
