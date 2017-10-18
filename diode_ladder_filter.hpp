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

#pragma once

#include <cmath>
#include <moditone/math/constants.hpp>
#include <functional>
#include <stdexcept>
#include <moditone/unit/hertz.hpp>

#include "analog_one_pole_filter.hpp"

namespace dsp
{
    //! Topology preserving 4-pole diode ladder filter with resolved zero delay feedback.
    /*! The ladder filter contains four stages resulting in a slope of 24 dB per octave.
     See "Designing software synthesizer plug-ins in c++" by Will Pirkle.
     See "The Art Of VA Filter Design" by Vadim Zavalishin. */
    template <class T>
    class DiodeLadderFilter
    {
    public:
        //! Construct the filter with a cut-off and sample rate
        DiodeLadderFilter(unit::hertz<float> cutOff, unit::hertz<float> sampleRate) :
        cutOff(cutOff),
        sampleRate(sampleRate)
        {
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
        
        //! Set the sample rate
        void setSampleRate(unit::hertz<float> sampleRate)
        {
            this->sampleRate = sampleRate;
            setCoefficients(cutOff, feedbackFactor, sampleRate);
        }
        
        //! Set coefficients given a cut-off, feedback factor and sample rate
        void setCoefficients(unit::hertz<float> cutOff, float feedbackFactor, unit::hertz<float> sampleRate)
        {
            this->feedbackFactor = feedbackFactor;
            g = std::tan(math::PI<T> * cutOff.value / sampleRate.value);
            auto gainFactorOnePole = g / (1.0 + g);

            // set one pole gain factor (G = g/(1+g))
            stage1.filter.setCutOffGain(gainFactorOnePole);
            stage2.filter.setCutOffGain(gainFactorOnePole);
            stage3.filter.setCutOffGain(gainFactorOnePole);
            stage4.filter.setCutOffGain(gainFactorOnePole);


            // global G's
            G4 = (0.5 * g) / (1 + g);
            G3 = (0.5 * g) / (1 + g - 0.5 * g * G4);
            G2 = (0.5 * g) / (1 + g - 0.5 * g * G3);
            G1 = g / (1 + g - g * G2);


            //// set stage factors

            // a0
            stage1.a0 = 1;
            stage2.a0 = 0.5;
            stage3.a0 = 0.5;
            stage4.a0 = 0.5;

            // gamma
            stage1.gamma = 1 + G1 * G2;
            stage2.gamma = 1 + G2 * G3;
            stage3.gamma = 1 + G3 * G4;
            stage4.gamma = 1;

            // epsilon
            stage1.epsilon = G2;
            stage2.epsilon = G3;
            stage3.epsilon = G4;
            stage4.epsilon = 0; // comment this out later

            // beta, denominators same at the G's... ja man doe hier iets mee ofzo
            stage1.beta = 1 / (1 + g - g * G2);
            stage2.beta = 1 / (1 + g - 0.5 * g * G3);
            stage3.beta = 1 / (1 + g - 0.5 * g * G4);
            stage4.beta = 1 / (1 + g);

            // delta
            stage1.delta = g;
            stage2.delta = 0.5 * g;
            stage3.delta = 0.5 * g;
            stage4.delta = 0; // comment this out later

            // feedback-factor
            stage1.feedbackFactor = G4 * G3 * G2;
            stage2.feedbackFactor = G4 * G3;
            stage3.feedbackFactor = G4;
            stage4.feedbackFactor = 1;

            cutOffGain = 1.0 / (1.0 + feedbackFactor * G4 * G3 * G2 * G1);
        }
        
        //! Write a sample to the filter
        void write(const T& x)
        {
            
            double S4 = stage4.filter.getIntegratorState() * stage4.beta;
            double addBefore4 = S4 * stage4.epsilon;
            
            double S3 = (stage3.filter.getIntegratorState() + S4 * stage3.delta) * stage3.beta;
            double addBefore3 = S3 * stage3.epsilon + S4;
            
            double S2 = (stage2.filter.getIntegratorState() + S3 * stage2.delta) * stage2.beta;
            double addBefore2 = S2 * stage2.epsilon + S3;
            
            double S1 = (stage1.filter.getIntegratorState() + S2 * stage1.delta) * stage1.beta;
            double addBefore1 = S1 * stage1.epsilon + S2;
            
            double feedbackSum =
            stage1.feedbackFactor * S1 +
            stage2.feedbackFactor * S2 +
            stage3.feedbackFactor * S3 +
            stage4.feedbackFactor * S4;
            
            ladderInput = (x - feedbackFactor * feedbackSum) * cutOffGain;
            
            // Optional non-linear processing
            if (globalNonLinear)
                ladderInput = globalNonLinear(ladderInput);
            
            double x1 = ladderInput * stage1.gamma + addBefore1;
            stage1(x1 * stage1.a0);
            
            double x2 = stage1.output * stage2.gamma + addBefore2;
            stage2(x2 * stage2.a0);
            
            double x3 = stage2.output * stage3.gamma + addBefore3;
            stage3(x3 * stage3.a0);
            
            double x4 = stage3.output * stage4.gamma + addBefore4;
            stage4(x4 * stage4.a0);
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
        
        //! Set a function for non-linear processing (or nullptr for linear) within the filter stage
        /*! Add colour to the filter with a saturating function to shape the feedback.
         Use with caution as this migth blow up the filter.
         See globalNonLinear for a non-linear component before the first stage */
        void setNonLinearStage(std::function<T(const T&)> nonLinear)
        {
            stage1.filter.nonLinear = nonLinear;
            stage2.filter.nonLinear = nonLinear;
            stage3.filter.nonLinear = nonLinear;
            stage4.filter.nonLinear = nonLinear;
        }
        
    public:
        //! Function for global non-linear processing
        //! A non-linear can be placed just before the first stage
        std::function<T(const T&)> globalNonLinear;
        
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
            AnalogOnePoleFilter<T> filter;
            
            //! The output of the filter
            T output = 0;
            
            double gamma = 0;
            double a0 = 0;
            double epsilon = 0;
            double beta = 0;
            double delta = 0;
            
            //! The feedback factor for the resonance peak
            double feedbackFactor = 0;
        };
        
    private:
        //! The cut-off
        unit::hertz<float> cutOff = sampleRate.value * 0.25;
        
        //! The sample rate
        unit::hertz<float> sampleRate = 44100;
        
        //! Feedback factor for resonance
        T feedbackFactor = 0;
        
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
        
        double g = 0;
        //! Filter gain factor with resolved zero delay feedback
        T cutOffGain = 0;
        
        double G1 = 0;
        double G2 = 0;
        double G3 = 0;
        double G4 = 0;
    };
}

