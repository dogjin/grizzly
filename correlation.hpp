//
//  correlation.hpp
//  Grizzly
//
//  Created by Stijn Frishert on 01/06/17.
//
//

#ifndef GRIZZLY_CORRELATION_HPP
#define GRIZZLY_CORRELATION_HPP

#include <dsperados/math/linear/dot.hpp>

#include "convolution.hpp"

namespace dsp
{
    //! Compute the dot product of a kernel and buffer, slide the kernel and repeat
    template <typename KernelIterator, typename BufferIterator>
    std::vector<float> slidingDotProduct(KernelIterator kernelBegin, KernelIterator kernelEnd, BufferIterator bufferBegin, BufferIterator bufferEnd, int start)
    {
        const auto bufferSize = std::distance(bufferBegin, bufferEnd);
        const auto kernelSize = std::distance(kernelBegin, kernelEnd);
        
        std::vector<float> result(bufferSize - start);
        
        for (auto lag = start; lag < bufferSize; ++lag)
        {
            for (auto k = 0; k < kernelSize; ++k)
            {
                if ((lag + k < 0) || (lag + k >= bufferSize))
                    continue;
                
                result[lag - start] += (*(bufferBegin + lag + k) * (*(kernelBegin + k)));
            }
        }
        
        return result;
    }
}

#endif /* GRIZZLY_CORRELATION_HPP */
