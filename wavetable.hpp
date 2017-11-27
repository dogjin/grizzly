//
//  wavetable.hpp
//  Grizzly
//
//  Created by Stijn Frishert on 11/27/17.
//

#ifndef GRIZZLY_WAVETABLE_HPP
#define GRIZZLY_WAVETABLE_HPP

#include <moditone/math/interpolation.hpp>
#include <vector>

#include "generator.hpp"

namespace dsp
{
    template <typename T>
    class WavetableGenerator :
        public Generator<T>
    {
    public:
        using Generator<T>::Generator;
        
        T convert() final
        {
            if (size == 0)
                return T();
                
            auto phase = this->getPhase() + this->getPhaseOffset();
            phase -= static_cast<int>(phase);
            
            return math::linearInterpolation(begin, begin + size, phase * size, math::wrapAccess);
        }
        
    public:
        const T* begin = nullptr;
        std::size_t size = 0;
    };
}

#endif /* GRIZZLY_WAVETABLE_HPP */
