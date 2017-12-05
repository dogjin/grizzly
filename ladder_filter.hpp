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

#ifndef GRIZZLY_LADDER_FILTER_HPP
#define GRIZZLY_LADDER_FILTER_HPP

#include <cmath>
#include <moditone/math/constants.hpp>
#include <functional>
#include <stdexcept>
#include <moditone/unit/hertz.hpp>

#include "analog_one_pole_filter.hpp"
#include "topology_preserving_one_pole_filter.hpp"

namespace dsp
{
    //! Topology preserving 4-pole ladder filter with resolved zero delay feedback.
    /*! The ladder filter contains four stages resulting in a slope of 24 dB per octave.
     See "Designing software synthesizer plug-ins in c++" by Will Pirkle.
     See "The Art Of VA Filter Design" by Vadim Zavalishin. */
    template <class T>
    class LadderFilter
    {
    public:
        //! Construct the filter with a cut-off and sample rate
        LadderFilter(unit::hertz<float> sampleRate) :
        sampleRate(sampleRate)
        {
            setSampleRate(sampleRate);
        }
        
        //! Set the sample rate
        void setSampleRate(unit::hertz<float> sampleRate)
        {
            this->sampleRate = sampleRate;
            setCoefficients(cutOff, feedbackFactor, sampleRate);
        }
        
        //! Set the cut-off frequency
        void setCutOff(unit::hertz<float> cutOff)
        {
            this->cutOff = cutOff;
            setCoefficients(cutOff, feedbackFactor, sampleRate);
        }
        
        //! Set the feedback factor for a resonance peak (self-oscillation at >= 4)
        void setFeedback(float factor)
        {
            feedbackFactor = factor;
            setCoefficients(cutOff, feedbackFactor, sampleRate);
        }
        
        //! Set coefficients given a cut-off, feedback factor and sample rate
        void setCoefficients(unit::hertz<float> cutOff, float feedbackFactor, unit::hertz<float> sampleRate)
        {
            this->feedbackFactor = feedbackFactor;
            const double integratorGainFactor = std::tan(math::PI<T> * cutOff.value / sampleRate.value);
            const double gainFactorOnePole = integratorGainFactor / (1.0 + integratorGainFactor);
            
            stage1.filter.setGain(gainFactorOnePole);
            stage2.filter.setGain(gainFactorOnePole);
            stage3.filter.setGain(gainFactorOnePole);
            stage4.filter.setGain(gainFactorOnePole);
            
            stage1.feedbackFactor = gainFactorOnePole * gainFactorOnePole * gainFactorOnePole / (1.0 + integratorGainFactor);
            stage2.feedbackFactor = gainFactorOnePole * gainFactorOnePole / (1.0 + integratorGainFactor);
            stage3.feedbackFactor = gainFactorOnePole / (1.0 + integratorGainFactor);
            stage4.feedbackFactor = 1.0 / (1.0 + integratorGainFactor);
            
            cutOffGain = 1.0 / (1.0 + feedbackFactor * (gainFactorOnePole * gainFactorOnePole * gainFactorOnePole * gainFactorOnePole));
        }
        
        //! Write a sample to the filter
        void write(const T& x)
        {
            const double feedbackSum = stage1.feedbackFactor * stage1.filter.getState() +
            stage2.feedbackFactor * stage2.filter.getState() +
            stage3.feedbackFactor * stage3.filter.getState() +
            stage4.feedbackFactor * stage4.filter.getState();
            
            // Multiply cut-off gain with the input minus the feedback to get the input for the first stage
            ladderInput = passBandGain ? (x * (1.0 + feedbackFactor) - feedbackFactor * feedbackSum) * cutOffGain : (x - feedbackFactor * feedbackSum) * cutOffGain;
            
            // Optional non-linear processing
            if (nonLinear)
                ladderInput = nonLinear(ladderInput);
            
            stage1(ladderInput);
            stage2(stage1.output);
            stage3(stage2.output);
            stage4(stage3.output);
        }
        
        //! Read the low-pass output
        T readLowPass() const
        {
            return stage4.output;
        }
        
        //! Write and read the low-pass output
        T writeAndReadLowPass(const T& x)
        {
            write(x);
            return readLowPass();
        }
        
        //! Read the band-pass output
        T readBandPass() const
        {
            return 4 * stage2.output + -8 * stage3.output + 4 * stage4.output;
        }
        
        //! Write and read the band-pass output
        T writeAndReadBandPass(const T& x)
        {
            write(x);
            return readBandPass();
        }
        
        //! Read the high-pass output
        T readHighPass() const
        {
            return ladderInput + -4 * stage1.output + 6 * stage2.output + -4 * stage3.output + stage4.output;
        }
        
        //! Write and read the high-pass output
        T writeAndReadHighPass(const T& x)
        {
            write(x);
            return readHighPass();
        }
        
        //! Read the second-order low-pass output
        T readSecondOrderLowPass() const
        {
            return stage2.output;
        }
        
        //! Write and read the second-order low-pass output
        T writeAndReadSecondOrderLowPass(const T& x)
        {
            write(x);
            return readSecondOrderLowPass();
        }
        
        //! Read the second-order band-pass output
        T readSecondOrderBandPass() const
        {
            return 2 * stage1.output + -2 * stage2.output;
        }
        
        //! Write and read the second-order band-pass output
        T writeAndReadSecondOrderBandPass(const T& x)
        {
            write(x);
            return readSecondOrderBandPass();
        }
        
        //! Read the second-order high-pass output
        T readSecondOrderHighPass() const
        {
            return ladderInput -2 * stage1.output + stage2.output;
        }
        
        //! Write and read the second-order high-pass output
        T writeAndReadSecondOrderHighPass(const T& x)
        {
            write(x);
            return readSecondOrderHighPass();
        }
        
    public:
        //! Pass-band gain compensation
        /*! Compensate for the loss of gain when the feedback factor increases, used in ARP filter models */
        bool passBandGain = false;
        
        //! Function for non-linear processing
        std::function<T(const T&)> nonLinear;
        
    private:
        //! The filter stage
        /*! Each stage contains an one-pole filter with a slope of 6 dB per octave. */
        struct Stage
        {
        public:
            //! Calculate the stage output given an input sample
            void operator()(const T& input)
            {
                output = filter.writeAndReadLowPass(input);
            }
            
        public:
            //! The one-pole filter
//            AnalogOnePoleFilter<T> filter;
            TopologyPreservingOnePoleFilterNonLinearII<float> filter;
            
            //! The output of the filter
            T output = 0;
            
            //! The feedback factor for the resonance peak
            double feedbackFactor = 0;
        };
        
    private:
        //! The cut-off
        unit::hertz<float> cutOff = 0;
        
        //! The sample rate
        unit::hertz<float> sampleRate = 0;
        
        //! Feedback factor for resonance
        double feedbackFactor = 0;
        
        //! Filter stage 1
        Stage stage1;
        
        //! Filter stage 2
        Stage stage2;
        
        //! Filter stage 3
        Stage stage3;
        
        //! Filter stage 4
        Stage stage4;
        
        //! The input state before the first stage of the ladder
        T ladderInput = 0;
        
        //! Filter gain factor with resolved zero delay feedback
        double cutOffGain = 0;
    };
}

#endif

