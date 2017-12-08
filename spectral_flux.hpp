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

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <iterator>
#include <vector>

namespace dsp
{
    template <typename Iterator>
    typename std::iterator_traits<Iterator>::value_type computeSpectralFluxFrame(Iterator frameBegin, Iterator frameEnd, Iterator previousFrameBegin, double order)
    {
        typename std::iterator_traits<Iterator>::value_type sum = 0;
        for ( ; frameBegin != frameEnd; ++frameBegin, ++previousFrameBegin)
        {
            const auto d = *frameBegin - *previousFrameBegin;
            if (d != 0)
                sum += std::pow(std::abs(d), order);
        }
        
        return std::pow(sum, 1 / order);
    }
    
    template <typename Iterator>
    typename std::iterator_traits<Iterator>::value_type computePositiveSpectralFluxFrame(Iterator frameBegin, Iterator frameEnd, Iterator previousFrameBegin, double order)
    {
        typename std::iterator_traits<Iterator>::value_type sum = 0;
        for ( ; frameBegin != frameEnd; ++frameBegin, ++previousFrameBegin)
        {
            const auto d = *frameBegin - *previousFrameBegin;
            if (d > 0)
                sum += std::pow(std::abs(d), order);
        }
        
        return std::pow(sum, 1 / order);
    }
    
    template <typename Iterator>
    typename std::iterator_traits<Iterator>::value_type computeNegativeSpectralFluxFrame(Iterator frameBegin, Iterator frameEnd, Iterator previousFrameBegin, double order)
    {
        typename std::iterator_traits<Iterator>::value_type sum = 0;
        for ( ; frameBegin != frameEnd; ++frameBegin, ++previousFrameBegin)
        {
            const auto d = *frameBegin - *previousFrameBegin;
            if (d < 0)
                sum += std::pow(std::abs(d), order);
        }
        
        return std::pow(sum, 1 / order);
    }
    
    template <typename Iterator>
    typename std::iterator_traits<Iterator>::value_type computeDifferenceSpectralFluxFrame(Iterator frameBegin, Iterator frameEnd, Iterator previousFrameBegin, double order)
    {
        const auto p = computePositiveSpectralFluxFrame(frameBegin, frameEnd, previousFrameBegin, order);
        const auto n = computeNegativeSpectralFluxFrame(frameBegin, frameEnd, previousFrameBegin, order);
        return p - n;
    }
    
    template <typename Iterator>
    std::vector<float> computeSpectralFlux(Iterator begin, Iterator end, double order)
    {
        if (begin == end)
            return {};
        
        auto next = std::next(begin);
        if (next == end)
            return {};
        
        std::vector<float> flux;
        for (auto prev = begin; next != end; ++prev, ++next)
            flux.emplace_back(computeSpectralFluxFrame(std::begin(*next), std::end(*next), std::begin(*prev), order));
        
        return flux;
    }
    
    template <typename Iterator>
    std::vector<float> computePositiveSpectralFlux(Iterator begin, Iterator end, double order)
    {
        if (begin == end)
            return {};
        
        auto next = std::next(begin);
        if (next == end)
            return {};
        
        std::vector<float> flux;
        for (auto prev = begin; next != end; ++prev, ++next)
            flux.emplace_back(computePositiveSpectralFluxFrame(std::begin(*next), std::end(*next), std::begin(*prev), order));
        
        return flux;
    }
    
    template <typename Iterator>
    std::vector<float> computeNegativeSpectralFlux(Iterator begin, Iterator end, double order)
    {
        if (begin == end)
            return {};
        
        auto next = std::next(begin);
        if (next == end)
            return {};
        
        std::vector<float> flux;
        for (auto prev = begin; next != end; ++prev, ++next)
            flux.emplace_back(computeNegativeSpectralFluxFrame(std::begin(*next), std::end(*next), std::begin(*prev), order));
        
        return flux;
    }
    
    template <typename Iterator>
    std::vector<float> computeDifferenceSpectralFlux(Iterator begin, Iterator end, double order, bool onlyPositive)
    {
        if (begin == end)
            return {};
        
        auto next = std::next(begin);
        if (next == end)
            return {};
        
        std::vector<float> flux;
        for (auto prev = begin; next != end; ++prev, ++next)
        {
            const auto f = computeDifferenceSpectralFluxFrame(std::begin(*next), std::end(*next), std::begin(*prev), order);
            flux.emplace_back(onlyPositive && f < 0 ? 0 : f);
        }
        
        return flux;
    }
}

#endif /* GRIZZLY_SPECTRAL_FLUX_HPP */
