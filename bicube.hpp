//
//  bicube.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 20/11/2017.
//

#pragma once

#include "bicube_coefficients.hpp"

namespace dsp
{    
    //! A bicube using Direct Form I
    template <class T, class CoeffType = T>
    class BicubeDirectForm1
    {
    public:
        //! Compute a sample
        void write(const T& x)
        {
            y = static_cast<T>(x * coefficients.a0 + xz1 * coefficients.a1 + xz2 * coefficients.a2 + xz3 * coefficients.a3
                               - coefficients.b1 * yz1 - coefficients.b2 * yz2 - coefficients.b3 * yz3);
            
            // Update the delays
            xz3 = xz2;
            xz2 = xz1;
            xz1 = x;
            
            yz3 = yz2;
            yz2 = yz1;
            yz1 = y;
        }
        
        //! Insert a new sample in the Biquad
        T read() const { return y; }
        
        //! Write a new sample and read the output (in that order)
        T writeAndRead(const T& x)
        {
            write(x);
            return read();
        }
        
        //! Set the filter state
        void setState(const T& state)
        {
            xz1 = state;
            yz1 = state;
            xz2 = state;
            yz2 = state;
            xz3 = state;
            yz3 = state;
            y = state;
        }
        
        //! Clear the delay elements
        void reset()
        {
            setState(0);
        }
        
    public:
        //! The coefficients to the biquad
        BicubeCoefficients<CoeffType> coefficients;
        
    private:
        T y = 0; //! output
        T xz1 = 0; //!< 1-sample input delay
        T xz2 = 0; //!< 2-sample input delay
        T xz3 = 0; //!< 3-sample input delay
        T yz1 = 0; //!< 1-sample output delay
        T yz2 = 0; //!< 2-sample output delay
        T yz3 = 0; //!< 3-sample output delay
    };
}
