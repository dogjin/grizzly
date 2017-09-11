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

#ifndef GRIZZLY_COMPLEX_HPP
#define GRIZZLY_COMPLEX_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <iterator>
#include <limits>
#include <vector>

#include <moditone/math/constants.hpp>
#include <moditone/math/wrap.hpp>
#include <unit/radian.hpp>

namespace dsp
{
    //! Construct a complex buffer from a real one
    template <typename RealIterator, typename ComplexIterator>
    void convertRealsToComplex(RealIterator inBegin, RealIterator inEnd, ComplexIterator outBegin)
    {
        std::transform(inBegin, inEnd, outBegin, [](const auto& x){ return static_cast<typename ComplexIterator::value_type>(x); });
    }
    
    //! Construct a complex buffer from a real one
    template <typename RealIterator>
    std::vector<std::complex<typename RealIterator::value_type>> convertRealsToComplex(RealIterator begin, RealIterator end)
    {
        std::vector<std::complex<typename RealIterator::value_type>> complex(std::distance(begin, end));
        convertRealsToComplex(begin, end, complex.begin());
        return complex;
    }
    
    //! Construct a complex buffer from a real one
    template <typename RealIterator, typename ComplexIterator>
    void convertImaginariesToComplex(RealIterator inBegin, RealIterator inEnd, ComplexIterator outBegin)
    {
        std::transform(inBegin, inEnd, outBegin, [](const auto& x){ return typename ComplexIterator::value_type(0, x); });
    }
    
    //! Construct a complex buffer from a real one
    template <typename RealIterator>
    std::vector<std::complex<typename RealIterator::value_type>> convertImaginariesToComplex(RealIterator begin, RealIterator end)
    {
        std::vector<std::complex<typename RealIterator::value_type>> complex;
        convertImaginariesToComplex(begin, end, complex.begin());
        return complex;
    }
    
    //! Return the real parts of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type::value_type> getReals(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<typename ComplexIterator::value_type::value_type> reals(std::distance(begin, end));
        std::transform(begin, end, reals.begin(), [&](auto x){ return x.real(); });
        return reals;
    }
    
    //! Return the imaginary parts of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type::value_type> getImaginaries(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<typename ComplexIterator::value_type::value_type> imaginaries(std::distance(begin, end));
        std::transform(begin, end, imaginaries.begin(), [&](auto x){ return x.imag(); });
        return imaginaries;
    }
    
    //! Take the abs of a vector of complex numbers
    template <typename InputIterator, typename OutputIterator>
    void computeMagnitudes(InputIterator inBegin, InputIterator inEnd, OutputIterator outBegin)
    {
        std::transform(inBegin, inEnd, outBegin, [](const auto& x){ return std::abs(x); });
    }
    
    //! Return the magnitudes of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type::value_type> computeMagnitudes(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<typename ComplexIterator::value_type::value_type> magnitudes(std::distance(begin, end));
        computeMagnitudes(begin, end, magnitudes.begin());
        return magnitudes;
    }
    
    //! Return the phases of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<unit::radian<typename ComplexIterator::value_type::value_type>> computePhases(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<unit::radian<typename ComplexIterator::value_type::value_type>> phases(std::distance(begin, end));
        std::transform(begin, end, phases.begin(), [&](auto x){ return std::arg(x); });
        return phases;
    }
    
    //! Return the unwrapped phases of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<unit::radian<typename ComplexIterator::value_type::value_type>> computeUnwrappedPhases(ComplexIterator begin, ComplexIterator end)
    {
        using T = typename ComplexIterator::value_type::value_type;
        
        // Retrieve the phases
        auto phases = computePhases(begin, end);
        math::unwrap(phases.begin(), phases.end(), -math::PI<T>, math::PI<T>);
        return phases;
    }
    
    //! Replace the real parts of a vector of complex numbers
    template <typename RealIterator, typename ComplexIterator>
    void replaceReals(RealIterator realBegin, RealIterator realEnd, ComplexIterator complexBegin)
    {
        while (realBegin != realEnd)
        {
            complexBegin->real(static_cast<typename ComplexIterator::value_type::value_type>(*realBegin));
            std::advance(complexBegin, 1);
            std::advance(realBegin, 1);
        }
    }
    
    //! Replace the imaginary parts of a vector of complex numbers
    template <typename RealIterator, typename ComplexIterator>
    void replaceImaginaries(RealIterator imagBegin, RealIterator imagEnd, ComplexIterator complexBegin)
    {
        while (imagBegin != imagEnd)
        {
            complexBegin->imag(static_cast<typename ComplexIterator::value_type::value_type>(*imagBegin));
            std::advance(complexBegin, 1);
            std::advance(imagBegin, 1);
        }
    }
    
    //! Replace the magnitudes of a vector of complex numbers
    template <typename RealIterator, typename ComplexIterator>
    void replaceMagnitudes(RealIterator magBegin, RealIterator magEnd, ComplexIterator complexBegin)
    {
        while (magBegin != magEnd)
        {
            const auto& mag = static_cast<typename ComplexIterator::value_type::value_type>(*magBegin);
            const auto angle = std::arg(*complexBegin);
            
            *complexBegin = (mag < 0) ? std::polar(-mag, angle + math::HALF_PI<decltype(angle)>) : std::polar(mag, angle);
            std::advance(complexBegin, 1);
            std::advance(magBegin, 1);
        }
    }
    
    //! Replace the phases of a vector of complex numbers
    template <typename RealIterator, typename ComplexIterator>
    void replacePhases(RealIterator phaseBegin, RealIterator phaseEnd, ComplexIterator complexBegin)
    {
        while (phaseBegin != phaseEnd)
        {
            const auto& mag = std::abs(*complexBegin);
            const auto angle = static_cast<typename ComplexIterator::value_type::value_type>(phaseBegin->value);
            
            *complexBegin = std::polar(std::abs(*complexBegin), angle);
            std::advance(complexBegin, 1);
            std::advance(phaseBegin, 1);
        }
    }
    
    //! Unwrap the phases of a collection of complex numbers in-place
    template <typename ComplexIterator>
    void unwrapPhases(ComplexIterator begin, ComplexIterator end)
    {
        const auto phases = unwrappedPhases(begin, end);
        replacePhases(phases.begin(), phases.end(), begin);
    }
    
    //! Take the log of a vector of complex numbers
    template <typename ComplexIterator>
    void computeLogs(ComplexIterator inBegin, ComplexIterator inEnd, ComplexIterator outBegin)
    {
        using T = typename ComplexIterator::value_type::value_type;
        std::transform(inBegin, inEnd, outBegin, [](const auto& x) -> typename ComplexIterator::value_type
        {
            return (x == T(0)) ? std::numeric_limits<T>::lowest() : std::log(x);
        });
    }
    
    //! Take the log of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type> computeLogs(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<typename ComplexIterator::value_type> logs(std::distance(begin, end));
        computeLogs(begin, end, logs.begin());
        return logs;
    }
    
    // Convert real data to complex
    template <typename Iterator, typename complexIterator>
    void convertRealToComplex(Iterator inBegin, Iterator inEnd, complexIterator outBegin)
    {
        std::transform(inBegin, inEnd, outBegin, [](const auto& x){ return std::complex<typename complexIterator::value_type::value_type>(x, 0); });
    }
    
    // Convert real data to complex
    template <typename Iterator>
    std::vector<std::complex<typename Iterator::value_type>> convertRealToComplex(Iterator begin, Iterator end)
    {
        std::vector<std::complex<typename Iterator::value_type>> out(std::distance(begin, end));
        convertRealToComplex(begin, end, out.begin());
        return out;
    }
    
//    //! Take the abs of a vector of complex numbers
//    template <typename ComplexIterator>
//    std::vector<typename ComplexIterator::value_type> computeMagnitudes(ComplexIterator begin, ComplexIterator end)
//    {
//        std::vector<typename ComplexIterator::value_type> out(std::distance(begin, end));
//        abs(begin, end, out.begin());
//        return out;
//    }
}

#endif /* GRIZZLY_COMPLEX_HPP */
