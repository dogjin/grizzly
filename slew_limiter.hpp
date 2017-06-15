//
//  slew_limiter.hpp
//  Grizzly
//
//  Created by Stijn Frishert on 15/06/17.
//
//

#ifndef GRIZZLY_SLEW_LIMITER_HPP
#define GRIZZLY_SLEW_LIMITER_HPP

#include <algorithm>

namespace dsp
{
    //! Moves a value towards a destination with a maximal slope
    /*! The slew limiter can be set to update its own internal value towards a destination value,
        but only by moving towards it with a maximal step each write() call. This ensures smoothed
        out values instead of arbitrarily large jumps, effectively resulting in a low-pass filtered
        signal. */
    template <typename T>
    class SlewLimiter
    {
    public:
        //! Construct the limiter by giving the maximal slew and starting value
        SlewLimiter(const T& maximalSlew, const T& initialValue) :
            maximalSlew(maximalSlew),
            value(initialValue)
        {
            
        }
        
        //! Move the limiter towards a new destination value
        /*! If the difference between the destination value and the actual current value is bigger
            than the maximal slope, the value will not move more than the maximal slope. */
        void write(const T& destination)
        {
            auto diff = destination - value;
            diff = (diff < 0) ? std::max(diff, -maximalSlew) : std::min(diff, maximalSlew);
            
            value += diff;
        }
        
        //! Read the value the limiter currently holds internally
        T read() const
        {
            return value;
        }
        
        //! Call to write() and read() at the same time
        T writeAndRead(const T& destination)
        {
            write(destination);
            return read();
        }
        
        //! Update the maximal slope/slew
        void setMaximalSlew(const T& slew)
        {
            if (slew < 0)
                throw std::invalid_argument("slew has to be positive");
            
            maximalSlew = slew;
        }
        
        //! Retrieve the value set for the maximal slope/slew
        T getMaximalSlew() const
        {
            return maximalSlew;
        }
        
    private:
        //! The maximal slope/slew; value won't move more than this slew each step
        T maximalSlew = 1;
        
        //! The internal value of the limiter
        T value = 0;
    };
}

#endif /* GRIZZLY_SLEW_LIMITER_HPP */
