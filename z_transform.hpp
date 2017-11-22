/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/moditone/grizzly for more information.
 
 Copyright (C) 2016-2018 Dsperados <info@dsperados.com>
 
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

#pragma once

#include <complex>

namespace dsp
{
    /*! @brief Apply the Z-transform on an input sequence
     *
     *  @discussion Apply the Z-transform given a series of numbers.
     *  The result is a function that takes a normalized frequency. Plugging in a frequency
     *  in this function, results in the transer of a single frequency.
     *  Possible usage: Apply it on an impulse response of a filter. Plug in several frequencies
     *  and plot the abs() of the result to display the magnitudes.
     *
     *  @return A lambda with signature std::complex<Iterator::value_type>(Iterator::value_type)
     *
     *  @see zTransformpoleZero zTransformbiquad
     */
    template <typename Iterator>
    auto zTransform(Iterator begin, Iterator end)
    {
        return [=](typename Iterator::value_type angularFrequency)
        {
            std::complex<typename Iterator::value_type> accumulator(0, 0);
            size_t index = 0;
            
            for (auto it = begin; it != end; ++it)
                accumulator += *it * std::polar<typename Iterator::value_type>(1, -angularFrequency * index++);
            
            return accumulator;
        };
    }
    
    /*! @brief Apply the Z-transform on a first order filter difference equation.
     *
     *  @discussion Apply the Z-transform given first order filter coefficients.
     *  The result is a function that takes a normalized frequency. Plugging in a frequency
     *  in this function, results in the transer of a single frequency.
     *  Possible usage: Tesing a cetrain filter setting. Plug in several frequencies and plot
     *  the abs() of the result to display the magnitudes.
     *
     *  @return A lambda with signature std::complex<T>(T)
     *
     *  @see zTransform zTransformbiquad
     */
    template <typename T>
    auto zTransformpoleZero(T a0, T a1, T b1)
    {
        return [=](T angularFrequency)
        {
            return  (a0 + a1 * std::polar<T>(T(1), -angularFrequency)) /
            (T(1) + b1 * std::polar<T>(T(1), -angularFrequency));
        };
    }
    
    /*! @brief Apply the Z-transform on a biquad filter difference equation.
     *
     *  @discussion Apply the Z-transform given biquad filter coefficients.
     *  The result is a function that takes a normalized frequency. Plugging in a frequency
     *  in this function, results in the transer of a single frequency.
     *  Possible usage: Tesing a cetrain filter setting. Plug in several frequencies and plot
     *  the abs() of the result to display the magnitudes.
     *
     *  @return A lambda with signature std::complex<T>(T)
     *
     *  @see zTransform zTransformpoleZero
     */
    template <typename T>
    auto zTransformbiquad(T a0, T a1, T a2, T b1, T b2)
    {
        return [=](T angularFrequency)
        {
            return  (a0 + a1 * std::polar<T>(T(1), -angularFrequency) + a2 * std::polar<T>(T(1), T(-2) * angularFrequency)) /
            (T(1) + b1 * std::polar<T>(T(1), -angularFrequency) + b2 * std::polar<T>(T(1), T(-2) * angularFrequency));
        };
    }
}

