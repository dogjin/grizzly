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

#include <dsperados/math/constants.hpp>
#include <dsperados/math/utility.hpp>
#include <unit/radian.hpp>

namespace dsp
{    
    //! Return the real parts of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type::value_type> real(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<typename ComplexIterator::value_type::value_type> real(std::distance(begin, end));
        std::transform(begin, end, real.begin(), [&](auto bin){ return bin.real(); });
        return real;
    }
    
    //! Return the imaginary parts of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type::value_type> imaginary(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<typename ComplexIterator::value_type::value_type> imaginary(std::distance(begin, end));
        std::transform(begin, end, imaginary.begin(), [&](auto bin){ return bin.imag(); });
        return imaginary;
    }
    
    //! Return the magnitudes of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type::value_type> magnitudes(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<typename ComplexIterator::value_type::value_type> magnitudes(std::distance(begin, end));
        std::transform(begin, end, magnitudes.begin(), [&](auto bin){ return std::abs(bin); });
        return magnitudes;
    }
    
    //! Return the phases of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<unit::radian<typename ComplexIterator::value_type::value_type>> phases(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<unit::radian<typename ComplexIterator::value_type::value_type>> phases(std::distance(begin, end));
        std::transform(begin, end, phases.begin(), [&](auto bin){ return std::arg(bin); });
        return phases;
    }
    
    //! Return the unwrapped phases of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<unit::radian<typename ComplexIterator::value_type::value_type>> unwrappedPhases(ComplexIterator begin, ComplexIterator end)
    {
        using T = typename ComplexIterator::value_type::value_type;
        
        // Retrieve the phases
        auto phases_ = phases(begin, end);
        math::unwrap(phases_.begin(), phases_.end(), -math::PI<T>, math::PI<T>);
        return phases_;
    }
    
    //! Replace the real parts of a vector of complex numbers
    template <typename RealIterator, typename ComplexIterator>
    void replaceReal(RealIterator realBegin, RealIterator realEnd, ComplexIterator complexBegin)
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
    void replaceImaginary(RealIterator imagBegin, RealIterator imagEnd, ComplexIterator complexBegin)
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
    void log(ComplexIterator inBegin, ComplexIterator inEnd, ComplexIterator outBegin)
    {
        using T = typename ComplexIterator::value_type::value_type;
        std::transform(inBegin, inEnd, outBegin, [](const auto& x) -> typename ComplexIterator::value_type
        {
            return (x == T(0)) ? std::numeric_limits<T>::lowest() : std::log(x);
        });
    }
    
    //! Take the log of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type> log(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<typename ComplexIterator::value_type> logs(std::distance(begin, end));
        log(begin, end, logs.begin());
        return logs;
    }
    
    //! Take the abs of a vector of complex numbers
    template <typename ComplexIterator>
    void abs(ComplexIterator inBegin, ComplexIterator inEnd, ComplexIterator outBegin)
    {
        std::transform(inBegin, inEnd, outBegin, [](const auto& x){ return std::abs(x); });
    }
    
    //! Take the abs of a vector of complex numbers
    template <typename ComplexIterator>
    std::vector<typename ComplexIterator::value_type> abs(ComplexIterator begin, ComplexIterator end)
    {
        std::vector<typename ComplexIterator::value_type> out(std::distance(begin, end));
        abs(begin, end, out.begin());
        return out;
    }
}

#endif /* GRIZZLY_COMPLEX_HPP */
