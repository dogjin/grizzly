//
//  sallen_key_filter.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 27/11/2017.
//

#pragma once

#include <functional>

#include "topology_preserving_filter.hpp"
#include "topology_preserving_one_pole_filter.hpp"

namespace dsp
{
    template <typename T>
    class SallenKeyFilter :
    public TopologyPreservingFilter<T>
    {
    public:
        SallenKeyFilter(double sampleRate_Hz) :
        TopologyPreservingFilter<T>(sampleRate_Hz)
        {
            this->resonance = resonanceMinimum;
        }
        
        T writeAndRead(T x)
        {
            this->write(x);
            return read();
        }
        
        T read() const
        {
            return this->y;
        }
        
    protected:
        T y = 0;
        
        double feedbackFactorS2 = 0;
        double feedbackFactorS3 = 0;
        
        TopologyPreservingOnePoleFilter<T> onePole1;
        TopologyPreservingOnePoleFilter<T> onePole2;
        TopologyPreservingOnePoleFilter<T> onePole3;
        
        double resonanceReciprocal = 0;
        
    private:
        void setCoefficients(double sampleRate_Hz, double cutOff_Hz, double resonance) final
        {
            onePole1.setCoefficients(cutOff_Hz, sampleRate_Hz);
            onePole2.copyCoefficients(onePole1);
            onePole3.copyCoefficients(onePole1);
            
            const auto g = onePole1.g;
            const auto gain = onePole1.gain;
            
            if (resonance <= 0)
                this->resonance = resonanceMinimum;
                
            resonanceReciprocal = 1.0 / this->resonance;
            
            this->gainFactor = 1.0 / (1.0 - this->resonance * gain + this->resonance * gain * gain);
            
            setFeedBackFactors(g, gain, this->resonance);
        }
        
        virtual void setFeedBackFactors(double g, double gain, double resonance) = 0;
        
    private:
        const double resonanceMinimum = 0.0000001;
    };
    
    
    template <typename T>
    class SallenKeyLowPass :
    public SallenKeyFilter<T>
    {
    public:
        SallenKeyLowPass(double sampleRate_Hz) :
        SallenKeyFilter<T>(sampleRate_Hz)
        {
        }
        
        void write(T x) final
        {
            T lowPass2Input = this->gainFactor * (this->onePole1.writeAndReadLowPass(x) + this->onePole2.state * this->feedbackFactorS2 + this->onePole3.state * this->feedbackFactorS3);
            
            if (this->nonLinear)
                lowPass2Input = this->nonLinear(lowPass2Input);
            
            this->y = this->resonance * this->onePole2.writeAndReadLowPass(lowPass2Input);
            
            this->onePole3.write(this->y);
            
            this->y *= this->resonanceReciprocal;
        }
        
    private:
        void setFeedBackFactors(double g, double gain, double resonance) final
        {
            this->feedbackFactorS2 = (resonance - resonance * gain) / (1.0 + g);
            this->feedbackFactorS3 = -1.0 / (1.0 + g);
        }
    };
    
    
    template <typename T>
    class SallenKeyHighPass :
    public SallenKeyFilter<T>
    {
    public:
        SallenKeyHighPass(double sampleRate_Hz) :
        SallenKeyFilter<T>(sampleRate_Hz)
        {
        }
        
        void write(T x) final
        {
            this->y = (this->onePole1.writeAndReadHighPass(x) + this->onePole2.state * this->feedbackFactorS2 + this->onePole3.state * this->feedbackFactorS3) * this->gainFactor * this->resonance;
            
            if (this->nonLinear)
                this->y = this->nonLinear(this->y);
            
            this->onePole3.write(this->onePole2.writeAndReadHighPass(this->y));
            
            this->y *= this->resonanceReciprocal;
        }
        
    private:
        void setFeedBackFactors(double g, double gain, double resonance) final
        {
            this->feedbackFactorS2 = -gain / (1.0 + g);
            this->feedbackFactorS3 = 1.0 / (1.0 + g);
        }
    };
}
