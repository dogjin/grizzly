//
//  sallen_key_filter.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 27/11/2017.
//

#pragma once

#include <functional>

#include "topology_preserving_one_pole_filter.hpp"

namespace dsp
{
    template <typename T>
    class SallenKeyLowPass
    {
    public:
        SallenKeyLowPass(double sampleRate_Hz) :
        sampleRate_Hz(sampleRate_Hz)
        {
            
        }
        
        T writeAndRead(T x)
        {
            write(x);
            return read();
        }
        
        void write(T x)
        {
            T lowPass2Input = gainFactor * (onePole1.writeAndReadLowPass(x) + onePole2.getState() * feedbackFactorS2 + onePole3.getState() * feedbackFactorS3);
            
            if (nonLinear)
                lowPass2Input = nonLinear(lowPass2Input);
            
            y = resonance * onePole2.writeAndReadLowPass(lowPass2Input);
            
            onePole3.write(y);
            
            if (resonance > 0)
                y *= 1.0 / resonance;
        }
        
        T read() const
        {
            return y;
        }
        
        void setCoefficients(double sampleRate_Hz, double cutOff_Hz, double resonance)
        {
            const double g = std::tan(math::PI<T> * cutOff_Hz / sampleRate_Hz);
            gain = g / (1.0 + g);
            
            onePole1.setGain(gain);
            onePole2.setGain(gain);
            onePole3.setGain(gain);
            
            feedbackFactorS2 = (resonance - resonance * gain) / (1.0 + g);
            feedbackFactorS3 = -1.0 / (1.0 + g);
            
            gainFactor = 1.0 / (1.0 - resonance * gain + resonance * gain * gain);
        }
        
        //! Set the sample rate
        void setSampleRate(double sampleRate_Hz)
        {
            if (this->sampleRate_Hz == sampleRate_Hz)
                return;
            
            this->sampleRate_Hz = sampleRate_Hz;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
        //! Set the cut-off frequency
        void setCutOff(double cutOff_Hz)
        {
            if (this->cutOff_Hz == cutOff_Hz)
                return;
            
            this->cutOff_Hz = cutOff_Hz;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
        //! Set the feedback factor for a resonance peak (self-oscillation at >= 4)
        void setResonance(float factor)
        {
            if (resonance == factor)
                return;
            
            resonance = factor;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
    public:
        //! Function for non-linear processing
        std::function<T(const T&)> nonLinear;
        
    private:
        T y = 0;
        
        double sampleRate_Hz = 0;
        double cutOff_Hz = 0;
        
        double gain = 0; // resolved 1p meuk
        double resonance = 0.01;
        
        double feedbackFactorS2 = 0;
        double feedbackFactorS3 = 0;
        double gainFactor = 0;
        
        TopologyPreservingOnePoleFilterLinear<T> onePole1;
        TopologyPreservingOnePoleFilterLinear<T> onePole2;
        TopologyPreservingOnePoleFilterLinear<T> onePole3;
    };
    
    

    
    
    template <typename T>
    class SallenKeyHighPass
    {
    public:
        SallenKeyHighPass(double sampleRate_Hz) :
        sampleRate_Hz(sampleRate_Hz)
        {
            
        }
        
        T writeAndRead(T x)
        {
            write(x);
            return read();
        }
        
        void write(T x)
        {
            y = (onePole1.writeAndReadHighPass(x) + onePole2.getState() * feedbackFactorS2 + onePole3.getState() * feedbackFactorS3) * gainFactor * resonance;
            
            if (nonLinear)
                y = nonLinear(y);
            
            onePole3.write(onePole2.writeAndReadHighPass(y));
            
            if (resonance > 0)
                y *= 1.0 / resonance;
        }
        
        T read() const
        {
            return y;
        }
        
        void setCoefficients(double sampleRate_Hz, double cutOff_Hz, double resonance)
        {
            const double g = std::tan(math::PI<T> * cutOff_Hz / sampleRate_Hz);
            gain = g / (1.0 + g);
            
            onePole1.setGain(gain);
            onePole2.setGain(gain);
            onePole3.setGain(gain);
            
            feedbackFactorS2 = -gain / (1.0 + g);
            feedbackFactorS3 = 1.0 / (1.0 + g);
            
            gainFactor = 1.0 / (1.0 - resonance * gain + resonance * gain * gain);
        }
        
        //! Set the sample rate
        void setSampleRate(double sampleRate_Hz)
        {
            if (this->sampleRate_Hz == sampleRate_Hz)
                return;
            
            this->sampleRate_Hz = sampleRate_Hz;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
        //! Set the cut-off frequency
        void setCutOff(double cutOff_Hz)
        {
            if (this->cutOff_Hz == cutOff_Hz)
                return;
            
            this->cutOff_Hz = cutOff_Hz;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
        //! Set the feedback factor for a resonance peak (self-oscillation at >= 4)
        void setResonance(float factor)
        {
            if (resonance == factor)
                return;
            
            resonance = factor;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
    public:
        //! Function for non-linear processing
        std::function<T(const T&)> nonLinear;
        
    private:
        T y = 0;
        
        double sampleRate_Hz = 0;
        double cutOff_Hz = 0;
        
        double gain = 0; // resolved 1p meuk
        double resonance = 0.01;
        
        double feedbackFactorS2 = 0;
        double feedbackFactorS3 = 0;
        double gainFactor = 0;
        
        TopologyPreservingOnePoleFilterLinear<T> onePole1;
        TopologyPreservingOnePoleFilterLinear<T> onePole2;
        TopologyPreservingOnePoleFilterLinear<T> onePole3;
    };
}
