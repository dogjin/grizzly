//
//  sliding_dot_product.hpp
//  Grizzly
//
//  Created by Stijn Frishert on 01/06/17.
//
//

#ifndef GRIZZLY_SLIDING_DOT_PRODUCT_HPP
#define GRIZZLY_SLIDING_DOT_PRODUCT_HPP

#include <moditone/math/linear/dot.hpp>
#include <iterator>
#include <vector>

namespace dsp
{
    //! Compute the dot product of a kernel and buffer, slide the kernel and repeat
    template <typename KernelIterator, typename BufferIterator>
    std::vector<float> computeSlidingDotProduct(KernelIterator kernelBegin, KernelIterator kernelEnd, BufferIterator bufferBegin, BufferIterator bufferEnd, int start)
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
                if ((lag + k >= 0) && (lag + k < bufferSize))
                    result[lag - start] += (*bufferIt) * (*kernelIt);
                
                std::advance(kernelIt, 1);
                std::advance(bufferIt, 1);
            }
        }
        
        return result;
    }
}

#endif /* GRIZZLY_SLIDING_DOT_PRODUCT_HPP */
