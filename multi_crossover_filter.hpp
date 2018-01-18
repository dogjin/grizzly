/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/moditone/grizzly for more information.
 
 Copyright (C) 2016-2018 Moditone <info@moditone.com>
 
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
#include <vector>

#include "cascade.hpp"
#include "crossover_filter.hpp"

namespace dsp
{
    using MultiCrossoverFilterOrder = CrossoverFilterOrder;
    
    //! A Multi-Crossover Filter to separate bands with multiple crossover filters
    /*! The Multi-Crossover filter holds crossover filters which can be accessed individually. The input cascades
        through all the filters, using the high-passed output as the next input for each stage. Therefore, the cut-off
        frequencies of the filters should be set in an ascending way for proper use.
        TODO: check of de cascade hier niet overbodig is en gewoon alleen met de vector werken... */
    template <class T>
    class MultiCrossoverFilter
    {
    public:
        //! Construct the filter
        MultiCrossoverFilter(T sampleRate_Hz, MultiCrossoverFilterOrder order = MultiCrossoverFilterOrder::SECOND) :
        order(order),
        sampleRate_Hz(sampleRate_Hz)
        {
        }
        
        //! Write a value to the filters
        void write(T input)
        {
            cascade.write(input);
        }
        
        //! Read the output of a single band
        T readBand(std::size_t index) const
        {
            // There is one band more than the number of filters
            if (index > filters.size())
                throw std::out_of_range("MultiCrossoverFilter::readBand out of range");
            
            // Do a readHigh() on the last crossover for the highest band
            if (index == filters.size())
                return filters.back()->readHigh();
            
            return filters[index]->readLow();
        }
        
        //! Read all the bands
        std::vector<T> readBands() const
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
        void emplaceBack(T cutOff_Hz)
        {
            auto filter = std::make_unique<CrossoverFilter<T>>(cutOff_Hz, sampleRate_Hz, order);
            cascade.emplaceBack([&, ptr=filter.get()](T input){ ptr->write(input); return ptr->readHigh(); });
            filters.emplace_back(std::move(filter));
        }
        
        //! Emplace a crossover filter at a given position
        void emplace(T cutOff_Hz, std::size_t position)
        {
            auto filter = std::make_unique<CrossoverFilter<T>>(cutOff_Hz, sampleRate_Hz, order);
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
        void setCutOff(T cutOff_Hz, std::size_t position)
        {
            if (position >= filters.size())
                throw std::out_of_range("position for setCutOff out of range");
            
            return filters[position]->setCutOff(cutOff_Hz);
        }
        
        //! Set the order for all the filters
        void setOrder(MultiCrossoverFilterOrder order)
        {
            this->order = order;
            for (auto& filter : filters)
                filter->setOrder(order);
        }
        
        //! Set the sample rate
        void setSampleRate(T sampleRate_Hz)
        {
            this->sampleRate_Hz = sampleRate_Hz;
            for (auto& filter : filters)
                filter->setSampleRate(sampleRate_Hz);
        }
        
        //! Return a const crossover filter
        const CrossoverFilter<T>& operator[](std::size_t index) const { return *filters[index]; }
        
        // Begin and end for ranged for-loops
        auto begin() { return filters.begin(); }
        auto begin() const { return filters.begin(); }
        auto end() { return filters.end(); }
        auto end() const { return filters.end(); }
        
    public:
        //! The order of the filter
        MultiCrossoverFilterOrder order = MultiCrossoverFilterOrder::SECOND;
        
        //! The sample rate used for all filters
        T sampleRate_Hz = 0;
        
        //! The cascade
        Cascade<T> cascade;
        
        //! The crossover filters
        std::vector<std::unique_ptr<CrossoverFilter<T>>> filters;
    };
}

#endif /* GRIZZLY_MULTI_CROSSOVER_FILTER_HPP */
