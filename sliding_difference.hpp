//
//  sliding_difference.hpp
//  Grizzly
//
//  Created by Stijn Frishert on 01/06/17.
//
//

#ifndef GRIZZLY_SLIDING_DIFFERENCE_HPP
#define GRIZZLY_SLIDING_DIFFERENCE_HPP

#include <iterator>
#include <vector>

namespace dsp
{
    template <typename KernelIterator, typename BufferIterator>
    std::vector<float> computeSlidingDifference(KernelIterator kernelBegin, KernelIterator kernelEnd, BufferIterator bufferBegin, BufferIterator bufferEnd, int start)
    {
        const auto bufferSize = std::distance(bufferBegin, bufferEnd);
        const auto kernelSize = std::distance(kernelBegin, kernelEnd);
        
        std::vector<float> result(bufferSize - start);
        
        for (auto lag = start; lag < bufferSize; ++lag)
        {
            auto bufferIt = std::next(bufferBegin, lag);
            auto kernelIt = kernelBegin;
            for (auto k = 0; k < kernelSize; ++k)
            {
                auto x = 0;
                if ((lag + k >= 0) && (lag + k < bufferSize))
                    x = *bufferIt;
                
                result[lag - start] += std::pow(x - (*kernelIt), 2);
                
                std::advance(kernelIt, 1);
                std::advance(bufferIt, 1);
            }
        }
        
        return result;
    }
}

#endif /* GRIZZLY_SLIDING_DIFFERENCE_HPP */
