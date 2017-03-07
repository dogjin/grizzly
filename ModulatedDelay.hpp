//
//  ModulatedDelay.hpp
//  grizzly
//
//  Created by Stijn Frishert on 07/03/17.
//
//

#ifndef GRIZZLY_MODULATED_DELAY_HPP
#define GRIZZLY_MODULATED_DELAY_HPP

#include <cstddef>

#include "CombFilter.hpp"
#include "Sine.hpp"

namespace dsp
{
    template <typename T>
    class ModulatedDelay
    {
    public:
        ModulatedDelay(const std::size_t maxDelay) :
            combFilter(maxDelay)
        {
            
        }
        
        void write(const T& x)
        {
            combFilter.write(x, 0.5f, 0.5f);
        }
        
        T read() const { return y; }
        
        T writeAndRead(const T& x)
        {
            write(x);
            return read();
        }
        
    private:
        FeedBackCombFilter<T> combFilter;
        UnipolarSine<T> sine;
        
        T y;
    };
}

#endif /* GRIZZLY_MODULATED_DELAY_HPP */
