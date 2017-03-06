//
//  Triangle.hpp
//  grizzly
//
//  Created by Stijn Frishert on 06/03/17.
//
//

#ifndef GRIZZLY_TRIANGLE_HPP
#define GRIZZLY_TRIANGLE_HPP

#include <dsperados/math/utility.hpp>
#include <unit/hertz.hpp>

namespace dsp
{
    //! Generate a bipolar triangle wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateBipolarTriangle(Phase phase)
    {
        return 2 * std::fabs(2 * math::wrap<std::common_type_t<Phase, T>>(phase - 0.25, 0, 1) - 1) - 1;
    }
    
    //! Generate a unipolar triangle wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateUnipolarTriangle(Phase phase)
    {
        phase = math::wrap<Phase>(phase, 0, 1);
        return phase < 0.5 ? phase * 2 : (0.5 - (phase - 0.5)) * 2;
    }
    
    //! Generates a saw triangle
    template <typename T>
    class BipolarTriangle
    {
    public:
        //! Increment the phase of the triangle
        void increment(long double increment)
        {
            setPhase(phase + increment);
            recomputeY();
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
            recomputeY();
        }
        
    private:
        //! Recompute the most recently computed value
        void recomputeY()
        {
            y = dsp::generateBipolarTriangle<T>(phase);
        }
        
    private:
        //! The to be returned value from read
        T y;
        
        //! The current phase of the triangle (range 0-1)
        long double phase = 0;
    };
    
    //! Generates a triangle wave
    template <typename T>
    class UnipolarTriangle
    {
    public:
        //! Increment the phase of the saw
        void increment(long double increment)
        {
            setPhase(phase + increment);
            recomputeY();
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
            recomputeY();
        }
        
    private:
        //! Recompute the most recently computed value
        void recomputeY()
        {
            y = dsp::generateUnipolarTriangle<T>(phase);
        }
        
    private:
        //! The to be returned value from read
        T y;
        
        //! The current phase of the triangle (range 0-1)
        long double phase = 0;
    };
}

#endif /* GRIZZLY_TRIANGLE_HPP */
