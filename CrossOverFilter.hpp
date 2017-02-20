/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2016 Dsperados <info@dsperados.com>
 
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

#ifndef CROSSOVER_FILTER_HPP
#define CROSSOVER_FILTER_HPP

#include <functional>
#include <vector>
#include <unit/hertz.hpp>

#include "Biquad.hpp"
#include "Cascade.hpp"
#include "FirstOrderFilter.hpp"

namespace dsp
{
    //! Crossover Filter
    /*! The filter is switchable to first, second, fourth and eighth order and has seperate read functions for the low and high band. */
    template <class T, class CoeffType = double>
    class CrossoverFilter
    {
    public:
        //! Order type
        enum class Order
        {
            FIRST,
            SECOND,
            FOURTH,
            EIGHTH
        };
        
    public:
        //! Construct a crossover filter given a order, cut-off and sample-rate
        CrossoverFilter(Order order, unit::hertz<float> cutOff, unit::hertz<float> sampleRate) :
        order(order),
        cutOff(cutOff),
        sampleRate(sampleRate)
        {
            setOrder(order);
        }
        
        //! Set the order
        void setOrder(Order order)
        {
            this->order = order;
            
            size_t numberOfstagesInCascade = 0;
            
            switch (order)
            {
                case Order::FIRST:
                    setCoefficients();
                    return;
                    
                case Order::SECOND:
                    biquadLowPass.resize(1);
                    biquadHighPass.resize(1);
                    setCoefficients();
                    return;
                    
                case Order::FOURTH:
                    numberOfstagesInCascade = 2;
                    break;
                    
                case Order::EIGHTH:
                    numberOfstagesInCascade = 4;
                    break;
            }
            
            biquadLowPass.resize(numberOfstagesInCascade);
            biquadHighPass.resize(numberOfstagesInCascade);
            
            lowPassCascade.eraseAll();
            highPassCascade.eraseAll();
            
            for (auto i = 0; i < numberOfstagesInCascade; ++i)
            {
                lowPassCascade.emplaceBack([&, i](T in){ biquadLowPass[i].write(in); return biquadLowPass[i].read(); });
                highPassCascade.emplaceBack([&, i](T in){ biquadHighPass[i].write(in); return biquadHighPass[i].read(); });
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
        
        //! Write the input to the filters
        void write(const T& input)
        {
            switch (order)
            {
                case Order::FIRST:
                    firstOrderLowPass.write(input);
                    firstOrderHighPass.write(input);
                    break;
                    
                case Order::SECOND:
                    biquadLowPass.front().write(input);
                    biquadHighPass.front().write(input);
                    break;
                    
                case Order::FOURTH:
                    lowPassCascade.write(input);
                    highPassCascade.write(input);
                    break;
                    
                case Order::EIGHTH:
                    lowPassCascade.write(input);
                    highPassCascade.write(input);
                    break;
            }
        }
        
        //! Read the low band of the filter
        T readLow() const
        {
            switch (order)
            {
                case Order::FIRST:
                    return firstOrderLowPass.read();
                    
                case Order::SECOND:
                    return biquadLowPass.front().read();
                    
                case Order::FOURTH:
                    return lowPassCascade.readOutput();
                    
                case Order::EIGHTH:
                    return lowPassCascade.readOutput();
            }
        }
        
        //! read the high band of the filter
        T readHigh() const
        {
            switch (order)
            {
                case Order::FIRST:
                    return firstOrderHighPass.read();
                    
                case Order::SECOND:
                    return -biquadHighPass.front().read(); // invert high-pass band to keep the low and high in phase
                    
                case Order::FOURTH:
                    return highPassCascade.readOutput();
                    
                case Order::EIGHTH:
                    return highPassCascade.readOutput();
            }
        }
        
    private:
        //! Set the filter coefficients
        void setCoefficients()
        {
            switch (order)
            {
                case Order::FIRST:
                    dsp::lowPassOnePoleZero(firstOrderLowPass.coefficients, sampleRate, unit::hertz<float>(cutOff));
                    dsp::highPassOnePoleZero(firstOrderHighPass.coefficients, sampleRate, unit::hertz<float>(cutOff));
                    break;
                    
                case Order::SECOND:
                    dsp::lowPass(biquadLowPass.front().coefficients, sampleRate, cutOff, 0.5);
                    dsp::highPass(biquadHighPass.front().coefficients, sampleRate, cutOff, 0.5);
                    break;
                    
                case Order::FOURTH:
                    // low-pass coefficients
                    dsp::lowPass(biquadLowPass[0].coefficients, sampleRate, cutOff, std::sqrt(0.5));
                    dsp::lowPass(biquadLowPass[1].coefficients, sampleRate, cutOff, std::sqrt(0.5));
                    
                    // high-pass coefficients
                    dsp::highPass(biquadHighPass[0].coefficients, sampleRate, cutOff, std::sqrt(0.5));
                    dsp::highPass(biquadHighPass[1].coefficients, sampleRate, cutOff, std::sqrt(0.5));
                    break;
                    
                case Order::EIGHTH:
                    // low-pass coefficients
                    dsp::lowPass(biquadLowPass[0].coefficients, sampleRate, cutOff, 0.541);
                    dsp::lowPass(biquadLowPass[1].coefficients, sampleRate, cutOff, 1.307);
                    dsp::lowPass(biquadLowPass[2].coefficients, sampleRate, cutOff, 0.541);
                    dsp::lowPass(biquadLowPass[3].coefficients, sampleRate, cutOff, 1.307);
                    
                    // high-pass coefficients
                    dsp::highPass(biquadHighPass[0].coefficients, sampleRate, cutOff, 0.541);
                    dsp::highPass(biquadHighPass[1].coefficients, sampleRate, cutOff, 1.307);
                    dsp::highPass(biquadHighPass[2].coefficients, sampleRate, cutOff, 0.541);
                    dsp::highPass(biquadHighPass[3].coefficients, sampleRate, cutOff, 1.307);
                    break;
            }
        }
        
    private:
        //! The filter order
        Order order = Order::FIRST;
        
        //! The cut-off
        unit::hertz<float> cutOff = 1000;
        
        //! The sample-rate
        unit::hertz<float> sampleRate = 44100;
        
        //! A vector of low-pass biquad filters used for an oder higher than one
        std::vector<dsp::BiquadDirectFormI<T, CoeffType>> biquadLowPass;
        
        //! A vector of high-pass biquad filters used for an oder higher than one
        std::vector<dsp::BiquadDirectFormI<T, CoeffType>> biquadHighPass;
        
        //! A cascade for cascading low-pass biquads used for an order higher than two
        dsp::Cascade<T> lowPassCascade;
        
        //! A cascade for cascading high-pass biquads used for an order higher than two
        dsp::Cascade<T> highPassCascade;
        
        //! First-order filter for low-pass filtering
        dsp::FirstOrderFilter<T, CoeffType> firstOrderLowPass;
        
        //! First-order filter for high-pass filtering
        dsp::FirstOrderFilter<T, CoeffType> firstOrderHighPass;
    };
}

#endif /* CROSSOVER_FILTER_HPP */