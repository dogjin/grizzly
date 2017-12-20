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

#ifndef GRIZZLY_ENVELOPE_FOLLOWER_HPP
#define GRIZZLY_ENVELOPE_FOLLOWER_HPP

#include <cmath>

#include "biquad.hpp"
#include "biquad_coefficients.hpp"
#include "first_order_filter.hpp"
#include "first_order_filter_coefficients.hpp"

namespace dsp
{
    //! AttackReleaseEnvelopeFollower
    /*! Abstract base class for envelope followers that use an attack and release time */
    template <class T>
    class AttackReleaseEnvelopeFollower
    {
    public:
        //! Construct the Follower
        AttackReleaseEnvelopeFollower(float sampleRate_Hz) :
        sampleRate_Hz(sampleRate_Hz)
        {
        }
        
        //! Virtual destructor
        virtual ~AttackReleaseEnvelopeFollower() = default;
        
    public:
        //! Set the sample rate
        void setSampleRate(float sampleRate_Hz)
        {
            this->sampleRate_Hz = sampleRate_Hz;
            lowPassOnePole(attackCoefficients, sampleRate_Hz, attackTime_s, timeConstantFactor);
            lowPassOnePole(releaseCoefficients, sampleRate_Hz, releaseTime_s, timeConstantFactor);
        }
        
        //! Set the time-constant factor
        void setTimeConstantFactor(float factor)
        {
            timeConstantFactor = factor;
            lowPassOnePole(attackCoefficients, sampleRate_Hz, attackTime_s, factor);
            lowPassOnePole(releaseCoefficients, sampleRate_Hz, releaseTime_s, factor);
        }
        
        //! Set the attack time
        virtual void setAttackTime(float attackTime_s)
        {
            this->attackTime_s = attackTime_s;
            
            if (attackTime_s > 0)
                lowPassOnePole(attackCoefficients, sampleRate_Hz, attackTime_s, timeConstantFactor);
            else
                throughPass(attackCoefficients);
        }
        
        //! Set the release time
        virtual void setReleaseTime(float releaseTime_s)
        {
            this->releaseTime_s = releaseTime_s;
            
            if (releaseTime_s > 0)
                lowPassOnePole(releaseCoefficients, sampleRate_Hz, releaseTime_s, timeConstantFactor);
            else
                throughPass(releaseCoefficients);
        }
        
        //! Write the new input to the follower
        virtual void write(T x) = 0;
        
        //! Read the last computed value
        virtual T read() const = 0;
        
        //! Write a new sample and read the output (in that order)
        T writeAndRead(T x)
        {
            write(x);
            return read();
        }
        
        //! Set the envelope state directly
        virtual void setState(T y) = 0;
        
        //! Set the envelope state to zero
        void reset()
        {
            setState(0);
        }
        
    protected:
        //! The attack coefficients
        FirstOrderCoefficients<T> attackCoefficients;
        
        //! The release coefficients
        FirstOrderCoefficients<T> releaseCoefficients;
        
        //! The sample rate
        T sampleRate_Hz = 0;
        
        //! Time constant factor
        T timeConstantFactor = 5.f;
        
        //! The attack time
        T attackTime_s = 0;
        
        //! The release time
        T releaseTime_s = 0;
        
        //! The most recently computed value
        T y = 0;
    };
    
    //! Envelope detector based on an analog circuit with two resistors, a capacitor and a diode
    /*! The capacitor continuously discharges via the release resistor. Due to this design, the peak will not
     reach its maximal value. choosing a bigger release time makes this less noticeably. See "Investigation
     in Dynamic Range Compression" by Massberg. */
    template <class T>
    class EnvelopeFollowerRCR : public AttackReleaseEnvelopeFollower<T>
    {
    public:
        //! Construct the Follower
        using AttackReleaseEnvelopeFollower<T>::AttackReleaseEnvelopeFollower;
        
        //! Write the new input to the follower
        void write(T x) final
        {
            this->y = static_cast<T>(-this->releaseCoefficients.b1 * this->y + this->attackCoefficients.a0 * std::max<T>(x - this->y, 0));
        }
        
        //! Read the last computed value
        T read() const final
        {
            return this->y;
        }
        
        //! Set the envelope state
        void setState(T y) final
        {
            this->y = y;
        }
    };
    
    //! Envelope detector based on an analog circuit with two resistor, two capacitors and a diode
    /*! The first set of resistor/capacitor is decoupled from the second set. In contrast to the EnvelopeDetectorRCR,
     the envelope will reach the maximal value regardless of different release settings. See "Investigation in Dynamic
     Range Compression" by Massberg. */
    template <class T>
    class EnvelopeFollowerDecoupled : public AttackReleaseEnvelopeFollower<T>
    {
    public:
        //! Construct the Follower
        using AttackReleaseEnvelopeFollower<T>::AttackReleaseEnvelopeFollower;
        
        //! Write the new input to the follower
        void write(T x) final
        {
            yRelease = std::max<T>(x, yRelease - this->releaseCoefficients.a0 * yRelease);
            this->y += this->attackCoefficients.a0 * (yRelease - this->y);
        }
        
        //! Read the last computed value
        T read() const final
        {
            return this->y;
        }
        
        //! Set the envelope state
        void setState(T y) final
        {
            this->y = y;
        }
        
    private:
        //! Intermediate release state
        T yRelease = 0;
    };
    
    //! Envelope detector using a one-pole low-pass filter
    /*! Notice the input is not rectified */
    template <class T>
    class EnvelopeFollowerOnePole : public AttackReleaseEnvelopeFollower<T>
    {
    public:
        //! Construct the Follower
        using AttackReleaseEnvelopeFollower<T>::AttackReleaseEnvelopeFollower;
        
        //! Write the new input to the follower
        void write(T x) final
        {
            if (x > lowPassFilter.read())
            {
                lowPassFilter.coefficients = this->attackCoefficients;
                lowPassFilter.write(x);
            }
            else
            {
                lowPassFilter.coefficients = this->releaseCoefficients;
                lowPassFilter.write(releaseToZero ? 0 : x);
            }
        }
        
        //! Read the last computed value
        T read() const final
        {
            return lowPassFilter.read();
        }
        
        //! Set the envelope state
        void setState(T y) final
        {
            lowPassFilter.setState(y);
        }
        
    public:
        //! Boolian for release mode
        /*! When true, the follower reaches for zero in its release state, otherwise the follower reaches to the input */
        bool releaseToZero = false;
        
    private:
        //! First order filter
        FirstOrderFilter<T, T> lowPassFilter;
    };
    
    //! Envelope detector using a biquad low-pass filter
    /*! Notice the input is not rectified */
    class EnvelopeFollowerBiquad
    {
    public:
        //! Construct the Follower
        EnvelopeFollowerBiquad(double sampleRate_Hz) :
        sampleRate_Hz(sampleRate_Hz)
        {
        }
        
        //! Write the new input to the follower
        void write(double x)
        {
            if (x > lowPassFilter.read())
            {
                lowPassFilter.coefficients = attackCoefficients;
                lowPassFilter.write(x);
            }
            else
            {
                lowPassFilter.coefficients = releaseCoefficients;
                lowPassFilter.write(releaseToZero ? 0.0 : x);
            }
        }
        
        //! Read the last computed value
        // WE DOEN DIT IN DOUBLES want afronding error enzo
        double read() const
        {
            return lowPassFilter.read();
        }
        
        double writeAndRead(double x)
        {
            write(x);
            return read();
        }
        
        //! Set the envelope state
        void setState(double y)
        {
            lowPassFilter.setState(y);
        }
        
        //! Set the attack time
        void setAttackTime(double attackTime_s)
        {
            this->attackTime_s = attackTime_s;
            
            if (attackTime_s > 0.0)
                lowPass<double>(attackCoefficients, sampleRate_Hz, attackTime_s, 0.5, timeConstantFactor);
            else
                throughPass(attackCoefficients);
        }
        
        //! Set the release time
        void setReleaseTime(double releaseTime_s)
        {
            this->releaseTime_s = releaseTime_s;
            
            if (releaseTime_s > 0.0)
                lowPass<double>(releaseCoefficients, sampleRate_Hz, releaseTime_s, 0.5, timeConstantFactor);
            else
                throughPass(releaseCoefficients);
        }
        
    public:
        //! Boolian for release mode
        /*! When true, the follower reaches for zero in its release state, otherwise the follower reaches to the input */
        bool releaseToZero = false;
        
    private:
        //! First order filter
        BiquadTransposedDirectForm2<double> lowPassFilter;
        
        BiquadCoefficients<double> attackCoefficients;
        BiquadCoefficients<double> releaseCoefficients;
        
        //! The sample rate
        double sampleRate_Hz = 0.0;
        
        //! Time constant factor
        double timeConstantFactor = 5.0;
        
        //! The attack time
        double attackTime_s = 0.0;
        
        //! The release time
        double releaseTime_s = 0.0;
        
        //! The most recently computed value
        double y = 0.0;
    };
}

#endif /* GRIZZLY_ENVELOPEDETECTOR_HPP */
