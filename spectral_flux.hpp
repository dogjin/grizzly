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

#ifndef GRIZZLY_SPECTRAL_FLUX_HPP
#define GRIZZLY_SPECTRAL_FLUX_HPP

#include <cstddef>
#include <iterator>
#include <vector>

namespace dsp
{
    template <typename Iterator>
    std::vector<float> spectralFlux(Iterator begin, Iterator end, std::size_t numberOfBins)
    {
        if (begin == end)
            return {};
        
        auto next = std::next(begin);
        if (next == end)
            return {};
        
        std::vector<float> flux;
        
        for (auto prev = begin; next != end; ++prev, ++next)
        {
            float sum = 0;
            for (auto i = 0; i < numberOfBins; ++i)
            {
                sum += std::norm((*next)[i] - (*prev)[i]);
            }
            
            flux.emplace_back(std::sqrt(sum));
        }
        
        return flux;
    }
    
    template <typename Iterator>
    std::vector<float> positiveSpectralFlux(Iterator begin, Iterator end, std::size_t numberOfBins)
    {
        if (begin == end)
            return {};
        
        auto next = std::next(begin);
        if (next == end)
            return {};
        
        std::vector<float> flux;
        
        for (auto prev = begin; next != end; ++prev, ++next)
        {
            float sum = 0;
            for (auto i = 0; i < numberOfBins; ++i)
            {
                const auto diff = std::abs((*next)[i]) - std::abs((*prev)[i]);
                if (diff <= 0)
                    continue;
                
                sum += diff;
            }
            
            flux.emplace_back(sum);
        }
        
        return flux;
    }
    
    template <typename Iterator>
    std::vector<float> negativeSpectralFlux(Iterator begin, Iterator end, std::size_t numberOfBins)
    {
        if (begin == end)
            return {};
        
        auto next = std::next(begin);
        if (next == end)
            return {};
        
        std::vector<float> flux;
        
        for (auto prev = begin; next != end; ++prev, ++next)
        {
            float sum = 0;
            for (auto i = 0; i < numberOfBins; ++i)
            {
                const auto diff = std::abs((*next)[i]) - std::abs((*prev)[i]);
                if (diff >= 0)
                    continue;
                
                sum -= diff;
            }
            
            flux.emplace_back(sum);
        }
        
        return flux;
    }
}

#endif /* GRIZZLY_SPECTRAL_FLUX_HPP */
