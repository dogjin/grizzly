/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2016-2017 Dsperados <info@dsperados.com>
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>
 
 --------------------------------------------------------------------
 
 If you would like to use Grizzly for commercial or closed-source
 purposes, please contact us for a commercial license.
 
 */

#ifndef GRIZZLY_BIQUAD_HPP
#define GRIZZLY_BIQUAD_HPP

#include "biquad_coefficients.hpp"

namespace dsp
{
    //! A biquad using Direct Form I
    /*! Biquad that computes samples using the Direct Form I topology.
     This topology gives you less side-effects when chaning coefficients during processing. */
    template <class T, class CoeffType = T>
    class BiquadDirectForm1
    {
    public:
        //! Compute a sample
        void write(const T& x)
        {
            y = static_cast<T>(x * coefficients.a0 + xz1 * coefficients.a1 + xz2 * coefficients.a2 - coefficients.b1 * yz1 - coefficients.b2 * yz2);
            
            // Update the delays
            xz2 = xz1;
            xz1 = x;
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
            y = state;
        }
        
        //! Clear the delay elements
        void reset()
        {
            setState(0);
        }
        
    public:
        //! The coefficients to the biquad
        BiquadCoefficients<CoeffType> coefficients;
        
    private:
        T y = 0; //! output
        T xz1 = 0; //!< 1-sample input delay
        T xz2 = 0; //!< 2-sample input delay
        T yz1 = 0; //!< 1-sample output delay
        T yz2 = 0; //!< 2-sample output delay
    };
    
    //! A biquad using Direct Form II
    /*! Biquad that computes samples using the Direct Form II topology.
     This topology minimizes the use of delays as it is shared between branches.
     A better floating-point accuracy is achieved in the transposed version. */
    template <class T, class CoeffType = T>
    class BiquadDirectForm2
    {
    public:
        //! Insert a new sample in the Biquad
        void write(const T& x)
        {
            auto v = x - coefficients.b1 * z1 - coefficients.b2 * z2;
            y = static_cast<T>(coefficients.a0 * v + coefficients.a1 * z1 + coefficients.a2 * z2);
            
            z2 = z1;
            z1 = v;
        }
        
        //! Insert a new sample in the Biquad
        T read() const { return y; }
        
        T writeAndRead(const T& x)
        {
            write(x);
            return read();
        }
        
        //! Set the filter state
        void setState(const T& state)
        {
            z1 = state;
            z1 = state;
            y = state;
        }
        
        //! Clear the delay elements
        void reset()
        {
            setState(0);
        }
        
    public:
        //! The coefficients to the biquad
        BiquadCoefficients<CoeffType> coefficients;
        
    private:
        T y = 0; //! output
        T z1 = 0; //!< 1-sample delay
        T z2 = 0; //!< 2-sample delay
    };
    
    //! A biquad using Transposed Direct Form I
    /*! Biquad that computes samples using the Transposed Direct Form I topology.
     Use transposed direct form II for better floating-point accuracy.
     Use direct form I for less side-effects when chaning coefficients during processing. */
    template <class T, class CoeffType = T>
    class BiquadTransposedDirectForm1
    {
    public:
        //! Compute a sample
        void write(const T& x)
        {
            auto v = x + yz1;
            yz1 = static_cast<T>(-coefficients.b1 * v + yz2);
            yz2 = static_cast<T>(-coefficients.b2 * v);
            
            y = static_cast<T>(coefficients.a0 * v + xz1);
            xz1 = static_cast<T>(coefficients.a1 * v + xz2);
            xz2 = static_cast<T>(coefficients.a2 * v);
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
            y = state;
        }
        
        //! Clear the delay elements
        void reset()
        {
            setState(0);
        }
        
    public:
        //! The coefficients to the biquad
        BiquadCoefficients<CoeffType> coefficients;
        
    private:
        T y = 0; //! output
        T xz1 = 0; //!< 1-sample input delay
        T xz2 = 0; //!< 2-sample input delay
        T yz1 = 0; //!< 1-sample output delay
        T yz2 = 0; //!< 2-sample output delay
    };
    
    //! A biquad using Transposed Direct Form II
    /*! Biquad that computes samples using the Transposed Direct Form II topology.
     This structes minimizes the use of delays and has a good floating-point accuracy,
     although it has more side-effects when you change the coefficients during processing. */
    template <class T, class CoeffType = T>
    class BiquadTransposedDirectForm2
    {
    public:
        //! Insert a new sample in the Biquad
        void write(const T& x)
        {
            y = static_cast<T>(x * coefficients.a0 + z1);
            
            // Update the delays
            z1 = static_cast<T>(x * coefficients.a1 + y * -coefficients.b1 + z2);
            z2 = static_cast<T>(x * coefficients.a2 + y * -coefficients.b2);
        }
        
        //! Insert a new sample in the Biquad
        T read() const { return y; }
        
        T writeAndRead(const T& x)
        {
            write(x);
            return read();
        }
        
        //! Set the filter state
        void setState(const T& state)
        {
            z1 = state;
            z1 = state;
            y = state;
        }
        
        //! Clear the delay elements
        void reset()
        {
            setState(0);
        }
        
    public:
        //! The coefficients to the biquad
        BiquadCoefficients<CoeffType> coefficients;
        
    private:
        T y = 0; //! output
        T z1 = 0; //!< 1-sample delay
        T z2 = 0; //!< 2-sample delay
    };
    
    template <class T, class CoeffType = T>
    using Biquad = BiquadTransposedDirectForm2<T, CoeffType>;
}

#endif
