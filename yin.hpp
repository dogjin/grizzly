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

#ifndef YinPitchDetection_hpp
#define YinPitchDetection_hpp

#include <cmath>
#include <vector>
#include <dsperados/math/interpolation.hpp>

namespace dsp
{
    //! Yin pitch tracking algoritm
    /*! Compute the pitch over a range of samples give a sampling rate and a threshold.
     Returns the pitch and probability in a std::pair.
     See "YIN, a fundamental frequency estimator for speech and music" by Alain de Cheveigne and Hideki Kawahara.
     http://audition.ens.fr/adc/pdf/2002_JASA_YIN.pdf
     */
    template <typename Iterator>
    std::pair<float, float> computeYin(Iterator begin, Iterator end, float sampleRate, float threshold = 0.15)
    {
        size_t size = std::distance(begin, end);
        size_t halfSize = size / 2;
        std::vector<float> differences(halfSize);
        float probability = 0;
        
        // Compute power difference of the frame, shifted to half of the size
        for (auto tau = 0 ; tau < halfSize; tau++)
            for (auto i = 0; i < halfSize; i++)
                differences[tau] += std::pow(begin[i] - begin[i + tau], 2);
        
        // Compute cumulative mean with first diffenrece set to one
        float sum = 0;
        differences.front() = 1;
        for (int tau = 1; tau < halfSize; tau++)
        {
            sum += differences[tau];
            differences[tau] *= tau / sum;
        }
        
        // Search local minima
        int tau;
        for (tau = 2; tau < halfSize; tau++)
        {
            // threshold reached
            if (differences[tau] < threshold)
            {
                // descend to local minimum and break, use this tau
                while (tau + 1 < halfSize && differences[tau + 1] < differences[tau])
                    tau++;
                
                probability = 1 - differences[tau];
                break;
            }
        }
        
        // No pitch found
        if (tau == halfSize || differences[tau] >= threshold)
            return {0, 0};
        
        // Apply parabolic interpolation
        size_t leftBound = math::clamp<size_t>(tau - 1, 0, halfSize - 1);
        size_t rightBound = math::clamp<size_t>(tau + 1, 0, halfSize - 1);
        
        if (leftBound == tau)
        {
            if (differences[tau] <= differences[rightBound])
                return {sampleRate / tau, probability};
            else
                return {sampleRate / rightBound, probability};
        }
        else if (rightBound == tau)
        {
            if (differences[tau] <= differences[leftBound])
                return {sampleRate / tau, probability};
            else
                return {sampleRate / leftBound, probability};
        }
        else // interpolate parabolic to get better local minimum, use the offset of the peak as correction on tau
            return {sampleRate / (tau + math::interpolateParabolic(differences[leftBound], differences[tau], differences[rightBound]).first), probability};
    }
}

#endif /* YinPitchDetection_hpp */
