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

#ifndef GRIZZLY_SPECTRUM_HPP
#define GRIZZLY_SPECTRUM_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <initializer_list>
#include <stdexcept>
#include <vector>

#include <dsperados/math/constants.hpp>
#include <dsperados/math/utility.hpp>
#include <unit/radian.hpp>

namespace dsp
{
    //! Spectrum of bins with a frequency and phase
    template <class T>
    class Spectrum
    {
    public:
        //! Utility type describing a bin
        using Bin = std::complex<T>;
        
    public:
        //! Default constructor
        Spectrum() = default;

        //! Construct a spectrum of a given number of bins
        Spectrum(std::size_t size) :
            bins(size)
        {
            
        }

        //! Construct a spectrum by providing the bins as iterators
        template <typename Iterator>
        Spectrum(Iterator begin, Iterator end) :
            bins(begin, end)
        {

        }

        //! Construct a spectrum by providing the bins directly
        Spectrum(std::initializer_list<Bin> bins) :
            bins{bins}
        {

        }
        
        //! Construct a spectrum with a vector of bins
        Spectrum(const std::vector<Bin>& bins) :
            bins(bins)
        {
            
        }
        
        //! Construct a spectrum with a vector of bins
        Spectrum(std::vector<Bin>&& bins) :
            bins(std::move(bins))
        {
            
        }

        //! Assign a vector of bins to the spectrum
        Spectrum& operator=(const std::vector<Bin>& bins)
        {
            this->bins = bins;
            return *this;
        }

        //! Assign a vector of bins to the spectrum
        Spectrum& operator=(std::vector<Bin>&& bins)
        {
            this->bins = std::move(bins);
            return *this;
        }
        
        //! Return the real part of the spectrum
        std::vector<T> real() const
        {
            std::vector<T> real(bins.size());
            std::transform(bins.begin(), bins.end(), real.begin(), [&](auto bin){ return bin.real(); });
            return real;
        }
        
        //! Return the imaginary part of the spectrum
        std::vector<T> imaginary() const
        {
            std::vector<T> imaginary(bins.size());
            std::transform(bins.begin(), bins.end(), imaginary.begin(), [&](auto bin){ return bin.imag(); });
            return imaginary;
        }
        
        //! Return the magnitudes of all bins
        std::vector<T> magnitudes() const
        {
            std::vector<T> magnitudes(bins.size());
            std::transform(bins.begin(), bins.end(), magnitudes.begin(), [&](auto bin){ return std::abs(bin); });
            return magnitudes;
        }
        
        //! Return the phases of all bins
        std::vector<unit::radian<T>> phases() const
        {
            std::vector<unit::radian<T>> phases(bins.size());
            std::transform(bins.begin(), bins.end(), phases.begin(), [&](auto bin){ return std::arg(bin); });
            return phases;
        }
        
        //! Return the unwrapped phases
        std::vector<unit::radian<T>> unwrappedPhases() const
        {
            // Retrieve the phases
            auto phases_ = phases();
            
            auto previousPhase = phases_[0];
            std::transform(phases_.begin() + 1, phases_.end(), phases_.begin() + 1, [&](auto phase)
            {
                while (phase.value - previousPhase.value < -math::PI<T>)
                    phase.value += math::PI<T>;
                
                while (phase.value - previousPhase.value >= math::PI<T>)
                    phase.value -= math::PI<T>;
                
                previousPhase = phase;
                return phase.value;
            });
            
            return phases_;
        }
        
        //! Replace the real part of the spectrum
        template <typename Iterator>
        void replaceReal(Iterator begin, Iterator end)
        {
            if (std::distance(begin, end) != bins.size())
                throw std::invalid_argument("iterator range is not of equal size");
            
            for (auto i = 0; i < bins.size(); ++i)
                bins[i].real(static_cast<T>(*begin++));
            assert(begin == end);
        }
        
        //! Replace the imaginary part of the spectrum
        template <typename Iterator>
        void replaceImaginary(Iterator begin, Iterator end)
        {
            if (std::distance(begin, end) != bins.size())
                throw std::invalid_argument("iterator range is not of equal size");
            
            for (auto i = 0; i < bins.size(); ++i)
                bins[i].imag(static_cast<T>(*begin++));
            assert(begin == end);
        }
        
        //! Replace the magnitudes of the spectrum
        template <typename Iterator>
        void replaceMagnitudes(Iterator begin, Iterator end)
        {
            if (std::distance(begin, end) != bins.size())
                throw std::invalid_argument("iterator range is not of equal size");
            
            for (auto i = 0; i < bins.size(); ++i)
                bins[i] = std::polar(static_cast<T>(*begin++), std::arg(bins[i]));
            assert(begin == end);
        }
        
        //! Replace the phases of the spectrum
        template <typename Iterator>
        void replacePhases(Iterator begin, Iterator end)
        {
            if (std::distance(begin, end) != bins.size())
                throw std::invalid_argument("iterator range is not of equal size");
            
            for (auto i = 0; i < bins.size(); ++i)
                bins[i] = std::polar(std::abs(bins[i]), static_cast<T>(*begin++));
            assert(begin == end);
        }

        //! Return a single bin in cartesian coordinates
        Bin& operator[](size_t index) { return bins[index]; }
        
        //! Return a single const bin in cartesian coordinates
        const Bin& operator[](size_t index) const { return bins[index]; }
        
        // Return iterators for ranged for-loops
        auto begin() { return bins.begin(); }
        auto begin() const { return bins.begin(); }
        auto end() { return bins.end(); }
        auto end() const { return bins.end(); }

        //! Return the size of the spectrum
        auto size() const { return bins.size(); }

        //! Return the data pointer of the spectrum
        T* data() { return bins.data(); }
        
        //! Return the data pointer of the spectrum
        const T* data() const { return bins.data(); }
        
    public:
        //! Spectrum in cartesian coordinates
        std::vector<Bin> bins;
    };
}

#endif /* GRIZZLY_SPECTRUM_HPP */
