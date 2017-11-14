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

#ifndef GRIZZLY_CROSSOVER_FILTER_HPP
#define GRIZZLY_CROSSOVER_FILTER_HPP

#include <cstddef>
#include <moditone/math/constants.hpp>
#include <functional>
#include <moditone/unit/hertz.hpp>
#include <vector>

#include "biquad.hpp"
#include "cascade.hpp"
#include "first_order_filter.hpp"

namespace dsp
{
    //! Order type
    enum class CrossoverFilterOrder
    {
        FIRST,
        SECOND,
        FOURTH,
        EIGHTH
    };
    
    //! Crossover Filter
    /*! Seperates low and high frequency bands, which added together result in the original signal (in magnitudes, not phases) */
    template <typename T, typename CoeffType = double>
    class CrossoverFilter
    {
    public:
        //! Construct a crossover filter given a order, cut-off and sample-rate
        CrossoverFilter(unit::hertz<float> cutOff, unit::hertz<float> sampleRate, CrossoverFilterOrder order = CrossoverFilterOrder::SECOND) :
            order(order),
            cutOff(cutOff),
            sampleRate(sampleRate)
        {
            setOrder(order);
        }
        
        //! Write the input to the filters
        void write(const T& input)
        {
            if (order == CrossoverFilterOrder::FIRST)
            {
                lowBand.firstOrderFilter.write(input);
                highBand.firstOrderFilter.write(input);
            }
            else
            {
                lowBand.cascade.write(input);
                highBand.cascade.write(input);
            }
        }
        
        //! Read the low band of the filter
        T readLow() const
        {
            return (order == CrossoverFilterOrder::FIRST) ? lowBand.firstOrderFilter.read() : lowBand.cascade.readOutput();
        }
        
        //! read the high band of the filter
        T readHigh() const
        {
            switch (order)
            {
                case CrossoverFilterOrder::FIRST:
                    return highBand.firstOrderFilter.read();
                case CrossoverFilterOrder::SECOND:
                    return -highBand.cascade.readOutput(); // invert high-pass band to keep the low and high in phase
                default:
                    return highBand.cascade.readOutput();
            }
        }
        
        //! Set the order
        void setOrder(CrossoverFilterOrder order)
        {
            this->order = order;
            
            lowBand.eraseFilters();
            highBand.eraseFilters();
            
            std::size_t numberOfStagesInCascade = 0;
            switch (order)
            {
                case CrossoverFilterOrder::FIRST: return setCoefficients();
                case CrossoverFilterOrder::SECOND: numberOfStagesInCascade = 1; break;
                case CrossoverFilterOrder::FOURTH: numberOfStagesInCascade = 2; break;
                case CrossoverFilterOrder::EIGHTH: numberOfStagesInCascade = 4; break;
            }
            
            lowBand.biquads.resize(numberOfStagesInCascade);
            highBand.biquads.resize(numberOfStagesInCascade);
            
            for (size_t i = 0; i < numberOfStagesInCascade; ++i)
            {
                lowBand.cascade.emplaceBack(&BiquadDirectForm1<T, CoeffType>::writeAndRead, &lowBand.biquads[i]);
                highBand.cascade.emplaceBack(&BiquadDirectForm1<T, CoeffType>::writeAndRead, &highBand.biquads[i]);
            }
            
            setCoefficients();
        }
        
        //! Set the cut-off
        void setCutOff(unit::hertz<float> cutOff)
        {
            this->cutOff = cutOff;
            setCoefficients();
        }
        
        //! Set the sample-rate
        void setSampleRate(unit::hertz<float> sampleRate)
        {
            this->sampleRate = sampleRate;
            setCoefficients();
        }
        
    private:
        //! Set the filter coefficients
        void setCoefficients()
        {
            switch (order)
            {
                case CrossoverFilterOrder::FIRST:
                    lowPassOnePoleZero(lowBand.firstOrderFilter.coefficients, sampleRate, cutOff);
                    highPassOnePoleZero(highBand.firstOrderFilter.coefficients, sampleRate, cutOff);
                    break;
                    
                case CrossoverFilterOrder::SECOND:
                    lowPass(lowBand.biquads[0].coefficients, sampleRate, cutOff, 0.5);
                    highPass(highBand.biquads[0].coefficients, sampleRate, cutOff, 0.5);
                    break;
                    
                case CrossoverFilterOrder::FOURTH:
                    // low-pass coefficients
                    lowPass(lowBand.biquads[0].coefficients, sampleRate, cutOff, math::SQRT_HALF<float>);
                    lowPass(lowBand.biquads[1].coefficients, sampleRate, cutOff, math::SQRT_HALF<float>);
                    
                    // high-pass coefficients
                    highPass(highBand.biquads[0].coefficients, sampleRate, cutOff, math::SQRT_HALF<float>);
                    highPass(highBand.biquads[1].coefficients, sampleRate, cutOff, math::SQRT_HALF<float>);
                    break;
                    
                case CrossoverFilterOrder::EIGHTH:
                    // low-pass coefficients according to Butterworth filters
                    lowPass(lowBand.biquads[0].coefficients, sampleRate, cutOff, 0.541f);
                    lowPass(lowBand.biquads[1].coefficients, sampleRate, cutOff, 1.307f);
                    lowPass(lowBand.biquads[2].coefficients, sampleRate, cutOff, 0.541f);
                    lowPass(lowBand.biquads[3].coefficients, sampleRate, cutOff, 1.307f);
                    
                    // high-pass coefficients according to Butterworth filters
                    highPass(highBand.biquads[0].coefficients, sampleRate, cutOff, 0.541f);
                    highPass(highBand.biquads[1].coefficients, sampleRate, cutOff, 1.307f);
                    highPass(highBand.biquads[2].coefficients, sampleRate, cutOff, 0.541f);
                    highPass(highBand.biquads[3].coefficients, sampleRate, cutOff, 1.307f);
                    break;
            }
        }
        
    private:
        //! Private structure for data per band
        class Band
        {
        public:
            //! First-order filter for order == 1
            FirstOrderFilter<T, CoeffType> firstOrderFilter;
            
            //! A vector of biquad filters used for an order >= 1
            std::vector<BiquadDirectForm1<T, CoeffType>> biquads;
            
            //! A cascade for chaining biquads used for an order >= 1
            Cascade<T> cascade;
            
        public:
            void eraseFilters()
            {
                cascade.eraseAll();
                biquads.clear();
            }
        };
        
    private:
        //! The filter order
        CrossoverFilterOrder order = CrossoverFilterOrder::SECOND;
        
        //! The cut-off
        unit::hertz<float> cutOff = 22050;
        
        //! The sample-rate
        unit::hertz<float> sampleRate = 44100;
        
        //! A band for low-passing the input signal
        Band lowBand;
        
        //! A band for high-passing the input signal
        Band highBand;
    };
}

#endif /* GRIZZLY_CROSSOVER_FILTER_HPP */
