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
#include <unit/hertz.hpp>
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
                lowPass.firstOrderFilter.write(input);
                highPass.firstOrderFilter.write(input);
            } else {
                lowPass.cascade.write(input);
                highPass.cascade.write(input);
            }
        }
        
        //! Read the low band of the filter
        T readLow() const
        {
            return (order == CrossoverFilterOrder::FIRST) ? lowPass.firstOrderFilter.read() : lowPass.cascade.readOutput();
        }
        
        //! read the high band of the filter
        T readHigh() const
        {
            switch (order)
            {
                case CrossoverFilterOrder::FIRST:
                    return highPass.firstOrderFilter.read();
                case CrossoverFilterOrder::SECOND:
                    return -highPass.cascade.readOutput(); // invert high-pass band to keep the low and high in phase
                default:
                    return highPass.cascade.readOutput();
            }
        }
        
        //! Set the order
        void setOrder(CrossoverFilterOrder order)
        {
            this->order = order;
            
            lowPass.eraseFilters();
            highPass.eraseFilters();
            
            std::size_t numberOfStagesInCascade = 0;
            switch (order)
            {
                case CrossoverFilterOrder::FIRST: return setCoefficients();
                case CrossoverFilterOrder::SECOND: numberOfStagesInCascade = 1; break;
                case CrossoverFilterOrder::FOURTH: numberOfStagesInCascade = 2; break;
                case CrossoverFilterOrder::EIGHTH: numberOfStagesInCascade = 4; break;
            }
            
            lowPass.biquads.resize(numberOfStagesInCascade);
            highPass.biquads.resize(numberOfStagesInCascade);
            
            for (size_t i = 0; i < numberOfStagesInCascade; ++i)
            {
                lowPass.cascade.emplaceBack(&dsp::BiquadDirectForm1<T, CoeffType>::writeAndRead, &lowPass.biquads[i]);
                highPass.cascade.emplaceBack(&dsp::BiquadDirectForm1<T, CoeffType>::writeAndRead, &highPass.biquads[i]);
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
                    dsp::lowPassOnePoleZero(lowPass.firstOrderFilter.coefficients, sampleRate, cutOff);
                    dsp::highPassOnePoleZero(highPass.firstOrderFilter.coefficients, sampleRate, cutOff);
                    break;
                    
                case CrossoverFilterOrder::SECOND:
                    dsp::lowPass(lowPass.biquads[0].coefficients, sampleRate, cutOff, 0.5);
                    dsp::highPass(highPass.biquads[0].coefficients, sampleRate, cutOff, 0.5);
                    break;
                    
                case CrossoverFilterOrder::FOURTH:
                    // low-pass coefficients
                    dsp::lowPass(lowPass.biquads[0].coefficients, sampleRate, cutOff, math::SQRT_HALF<float>);
                    dsp::lowPass(lowPass.biquads[1].coefficients, sampleRate, cutOff, math::SQRT_HALF<float>);
                    
                    // high-pass coefficients
                    dsp::highPass(highPass.biquads[0].coefficients, sampleRate, cutOff, math::SQRT_HALF<float>);
                    dsp::highPass(highPass.biquads[1].coefficients, sampleRate, cutOff, math::SQRT_HALF<float>);
                    break;
                    
                case CrossoverFilterOrder::EIGHTH:
                    // low-pass coefficients according to Butterworth filters
                    dsp::lowPass(lowPass.biquads[0].coefficients, sampleRate, cutOff, 0.541f);
                    dsp::lowPass(lowPass.biquads[1].coefficients, sampleRate, cutOff, 1.307f);
                    dsp::lowPass(lowPass.biquads[2].coefficients, sampleRate, cutOff, 0.541f);
                    dsp::lowPass(lowPass.biquads[3].coefficients, sampleRate, cutOff, 1.307f);
                    
                    // high-pass coefficients according to Butterworth filters
                    dsp::highPass(highPass.biquads[0].coefficients, sampleRate, cutOff, 0.541f);
                    dsp::highPass(highPass.biquads[1].coefficients, sampleRate, cutOff, 1.307f);
                    dsp::highPass(highPass.biquads[2].coefficients, sampleRate, cutOff, 0.541f);
                    dsp::highPass(highPass.biquads[3].coefficients, sampleRate, cutOff, 1.307f);
                    break;
            }
        }
        
    private:
        //! Private structure for data per band
        class Band
        {
        public:
            //! First-order filter for order == 1
            dsp::FirstOrderFilter<T, CoeffType> firstOrderFilter;
            
            //! A vector of biquad filters used for an order >= 1
            std::vector<dsp::BiquadDirectForm1<T, CoeffType>> biquads;
            
            //! A cascade for chaining biquads used for an order >= 1
            dsp::Cascade<T> cascade;
            
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
        Band lowPass;
        
        //! A band for high-passing the input signal
        Band highPass;
    };
}

#endif /* GRIZZLY_CROSSOVER_FILTER_HPP */
