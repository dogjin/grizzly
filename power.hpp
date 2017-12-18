//
//  power.hpp
//  Grizzly
//
//  Created by Stijn Frishert on 2017/12/14.
//

#ifndef GRIZZLY_POWER_HPP
#define GRIZZLY_POWER_HPP

#include <cmath>
#include <vector>

namespace dsp
{
    template <typename InputIterator, typename OutputIterator>
    void computePower(InputIterator inBegin, InputIterator inEnd, OutputIterator outBegin)
    {
        for (auto it = inBegin; it != inEnd; ++it)
            *outBegin++ = std::pow(*it, typename InputIterator::value_type(2));
    }
    
    template <typename Iterator>
    std::vector<typename Iterator::value_type> computePower(Iterator begin, Iterator end)
    {
        std::vector<typename Iterator::value_type> result(std::distance(begin, end));
        computePower(begin, end, result.begin());
        return result;
    }
}

#endif /* GRIZZLY_POWER_HPP */
