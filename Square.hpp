//
//  Square.hpp
//  grizzly
//
//  Created by Stijn Frishert on 06/03/17.
//
//

#ifndef GRIZZLY_SQUARE_HPP
#define GRIZZLY_SQUARE_HPP

#include "Waveform.hpp"

namespace dsp
{
    //! Generates a bipolar square wave
    template <typename T>
    class Square
    {
    public:
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
        
        //! The current phase of the sine (range 0-1)
        long double phase = 0;
        
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
}

#endif /* GRIZZLY_SQUARE_HPP */
