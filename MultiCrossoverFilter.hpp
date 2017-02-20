/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2017 Dsperados <info@dsperados.com>
 
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

#ifndef MULTI_CROSSOVER_FILTER_HPP
#define MULTI_CROSSOVER_FILTER_HPP

#include <functional>
#include <unit/hertz.hpp>
#include <vector>

#include "Cascade.hpp"
#include "CrossoverFilter.hpp"

namespace dsp
{
    //! A Multi-Crossover Filter to separate bands with multiple crossover filters
    /*! The Multi-Crossover filter holds crossover filters which can be accessed individually.
     The input cascades through all the filters, using the high-passed output as the next input for each stage.
     Therefor, the cut-off frequencies of the filters should be set in an ascending way for proper use. */
    template <class T, class CoeffType = double>
    class MultiCrossoverFilter
    {
    public:
        //! Set the sample rate
        void setSampleRate(unit::hertz<float> sampleRate)
        {
            for (auto& filter: filters)
                filter->setSampleRate(sampleRate);
        }
        
        //! Set the order for all the filters
        void setOrder(typename dsp::CrossoverFilter<T, CoeffType>::Order order)
        {
            for (auto& filter: filters)
                filter->setOrder(order);
        }
        
        //! Write a value to the filters
        void write(const T& input)
        {
            cascade.write(input);
        }
        
        //! Read al the bands
        std::vector<T> readBands()
        {
            std::vector<T> bands;
            
            if (filters.empty())
                return bands;
            
            // There is one band more than the number of filters
            auto numberOfBands = filters.size() + 1;
            bands.resize(numberOfBands);
            
            for (auto band = 0; band < numberOfBands; ++band)
                bands[band] = readBand(band);
            
            return bands;
        }
        
        //! Read the output of a single band
        T readBand(size_t index)
        {
            // There is one band more than the number of filters
            if (index > filters.size())
                throw std::invalid_argument("index > filters.size()");
            
            // Do a readHigh() on the last crossover for the highest band
            if (index == filters.size())
                return filters.back()->readHigh();
            
            return filters[index]->readLow();
        }
        
        //! Emplace back a crossover filter
        void emplaceBack(typename dsp::CrossoverFilter<T, CoeffType>::Order order, unit::hertz<float> cutOff, unit::hertz<float> sampleRate)
        {
            auto filter = std::make_unique<dsp::CrossoverFilter<T, CoeffType>>(order, cutOff, sampleRate);
            cascade.emplaceBack([&, ptr=filter.get()](T input){ ptr->write(input); return ptr->readHigh(); });
            filters.emplace_back(std::move(filter));
        }
        
        //! Emplace a crossover filter at a given position
        void emplace(typename dsp::CrossoverFilter<T, CoeffType>::Order order, unit::hertz<float> cutOff, unit::hertz<float> sampleRate, size_t position)
        {
            auto filter = std::make_unique<dsp::CrossoverFilter<T, CoeffType>>(order, cutOff, sampleRate);
            cascade.emplace([&, ptr=filter.get()](T input){ ptr->write(input); return ptr->readHigh(); }, position);
            filters.emplace(filters.begin() + position, std::move(filter));
        }
        
        //! Erase a crossover filter at a given position
        void erase(size_t position)
        {
            cascade.erase(position);
            filters.erase(filters.begin() + position);
        }
        
        //! Erase all crossover filters
        void eraseAll()
        {
            cascade.eraseAll();
            filters.clear();
        }
        
        //! Return begin iterator for ranged for-loops
        auto begin() { return filters.begin(); }
        
        //! Return const begin iterator for ranged for-loops
        auto begin() const { return filters.begin(); }
        
        //! Return end iterator for ranged for-loops
        auto end() { return filters.end(); }
        
        //! Return const end iterator for ranged for-loops
        auto end() const { return filters.end(); }
        
        //! Return a crossover filter
        dsp::CrossoverFilter<T, CoeffType>& operator[](size_t index){ return *filters[index]; };
        
        //! Return a const crossover filter
        const dsp::CrossoverFilter<T, CoeffType>& operator[](size_t index) const { return *filters[index]; };
        
    public:
        //! The cascade
        dsp::Cascade<T> cascade;
        
        //! The crossover filters
        std::vector<std::unique_ptr<dsp::CrossoverFilter<T, CoeffType>>> filters;
    };
}

#endif /* MULTI_CROSSOVER_FILTER_HPP */
