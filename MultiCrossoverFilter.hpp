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

#ifndef GRIZZLY_MULTI_CROSSOVER_FILTER_HPP
#define GRIZZLY_MULTI_CROSSOVER_FILTER_HPP

#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <unit/hertz.hpp>
#include <vector>

#include "Cascade.hpp"
#include "CrossoverFilter.hpp"

namespace dsp
{
    using MultiCrossoverFilterOrder = CrossoverFilterOrder;
    
    //! A Multi-Crossover Filter to separate bands with multiple crossover filters
    /*! The Multi-Crossover filter holds crossover filters which can be accessed individually. The input cascades
        through all the filters, using the high-passed output as the next input for each stage. Therefore, the cut-off
        frequencies of the filters should be set in an ascending way for proper use. */
    template <class T, class CoeffType = double>
    class MultiCrossoverFilter
    {
    public:
        //! Construct the filter
        MultiCrossoverFilter(unit::hertz<float> sampleRate, MultiCrossoverFilterOrder order = MultiCrossoverFilterOrder::SECOND) :
            order(order),
            sampleRate(sampleRate)
        {
            
        }
        
        //! Write a value to the filters
        void write(const T& input)
        {
            cascade.write(input);
        }
        
        //! Read the output of a single band
        T readBand(std::size_t index)
        {
            // There is one band more than the number of filters
            if (index > filters.size())
                throw std::out_of_range("MultiCrossoverFilter::readBand out of range");
            
            // Do a readHigh() on the last crossover for the highest band
            if (index == filters.size())
                return filters.back()->readHigh();
            
            return filters[index]->readLow();
        }
        
        //! Read al the bands
        std::vector<T> readBands()
        {
            // Return nothing if there are not bands
            std::vector<T> bands;
            if (filters.empty())
                return bands;
            
            // There is one band more than the number of filters
            bands.resize(filters.size() + 1);
            
            for (auto i = 0; i < bands.size(); ++i)
                bands[i] = readBand(i);
            
            return bands;
        }
        
        //! Emplace back a crossover filter
        void emplaceBack(unit::hertz<float> cutOff)
        {
            auto filter = std::make_unique<dsp::CrossoverFilter<T, CoeffType>>(cutOff, sampleRate, order);
            cascade.emplaceBack([&, ptr=filter.get()](T input){ ptr->write(input); return ptr->readHigh(); });
            filters.emplace_back(std::move(filter));
        }
        
        //! Emplace a crossover filter at a given position
        void emplace(unit::hertz<float> cutOff, std::size_t position)
        {
            auto filter = std::make_unique<dsp::CrossoverFilter<T, CoeffType>>(cutOff, sampleRate, order);
            cascade.emplace([&, ptr=filter.get()](T input){ ptr->write(input); return ptr->readHigh(); }, position);
            filters.emplace(filters.begin() + position, std::move(filter));
        }
        
        //! Erase a crossover filter at a given position
        void erase(std::size_t position)
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
        
        //! Change the cut-off for a given filter
        void setCutOff(unit::hertz<float> cutOff, std::size_t position)
        {
            if (position >= filters.size())
                throw std::out_of_range("position for setCutOff out of range");
            
            return filters[position]->setCutOff(cutOff);
        }
        
        //! Set the order for all the filters
        void setOrder(MultiCrossoverFilterOrder order)
        {
            this->order = order;
            for (auto& filter : filters)
                filter->setOrder(order);
        }
        
        //! Set the sample rate
        void setSampleRate(unit::hertz<float> sampleRate)
        {
            this->sampleRate = sampleRate;
            for (auto& filter : filters)
                filter->setSampleRate(sampleRate);
        }
        
        //! Return a const crossover filter
        const dsp::CrossoverFilter<T, CoeffType>& operator[](std::size_t index) const { return *filters[index]; }
        
        // Begin and end for ranged for-loops
        auto begin() { return filters.begin(); }
        auto begin() const { return filters.begin(); }
        auto end() { return filters.end(); }
        auto end() const { return filters.end(); }
        
    public:
        //! The order of the filter
        MultiCrossoverFilterOrder order = MultiCrossoverFilterOrder::SECOND;
        
        //! The sample rate used for all filters
        unit::hertz<float> sampleRate = 44100;
        
        //! The cascade
        dsp::Cascade<T> cascade;
        
        //! The crossover filters
        std::vector<std::unique_ptr<dsp::CrossoverFilter<T, CoeffType>>> filters;
    };
}

#endif /* GRIZZLY_MULTI_CROSSOVER_FILTER_HPP */
