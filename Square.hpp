//
//  Square.hpp
//  grizzly
//
//  Created by Stijn Frishert on 06/03/17.
//
//

#ifndef GRIZZLY_SQUARE_HPP
#define GRIZZLY_SQUARE_HPP

#include <dsperados/math/utility.hpp>

#include "Waveform.hpp"

namespace dsp
{
    //! Generate a square wave given a normalized phase
    template <typename T, typename Phase, typename PulseWidth>
    constexpr T generateSquare(Phase phase, PulseWidth pulseWidth, T low = 0, T high = 1)
    {
        return math::wrap<Phase>(phase, 0, 1) < pulseWidth ? high : low;
    }
    
    //! Generates a square wave
    template <typename T>
    class Square
    {
    public:
        Square(const T& min = -1, const T& max = 1) :
            min(min),
            max(max)
        {
            
        }
        
        //! Increment the phase of the square
        void increment(long double increment)
        {
            phase = setPhase(phase + increment);
            recomputeOutput();
        }
        
        //! Increment the phase, given a frequency
        void increment(unit::hertz<float> frequency, unit::hertz<float> sampleRate)
        {
            increment(frequency.value / sampleRate.value);
        }
        
        //! Read the most recently computed output
        T read() const
        {
            return y;
        }
        
        //! Change the phase manually
        void setPhase(long double phase)
        {
            this->phase = math::wrap<long double>(phase, 0, 1);
            recomputeOutput();
        }
        
        //! Change the pulse width
        /*! @param recompute: Recompute the y to return from read() */
        void setPulseWidth(float pulseWidth, bool recompute)
        {
            this->pulseWidth = pulseWidth;
            
            if (recompute)
                recomputeOutput();
        }
        
    private:
        //! Recompute the most recently computed value
        void recompute()
        {
            y = dsp::generateSquare<T>(phase, pulseWidth);
        }
        
    private:
        //! The to be returned value from read
        T y;
        
        //! The minimum value
        T min = -1;
        
        //! The maximum value;
        T max = 1;
        
        //! The current phase of the sine (range 0-1)
        long double phase = 0;
        
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
}

#endif /* GRIZZLY_SQUARE_HPP */
