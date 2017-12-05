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
            this->resonance = 0.00001;
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
        double gainFactor = 0;
        
        TopologyPreservingOnePoleFilterLinear<T> onePole1;
        TopologyPreservingOnePoleFilterLinear<T> onePole2;
        TopologyPreservingOnePoleFilterLinear<T> onePole3;
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
            
            if (this->resonance > 0)
                this->y *= 1.0 / this->resonance;
        }
        
        void setCoefficients(double sampleRate_Hz, double cutOff_Hz, double resonance) final
        {
            this->onePole1.setCoefficients(cutOff_Hz, sampleRate_Hz);
            
            auto g = this->onePole1.g;
            auto gain = this->onePole1.gain;
            
            this->onePole2.gain = gain;
            this->onePole3.gain = gain;
            
            this->feedbackFactorS2 = (resonance - resonance * gain) / (1.0 + g);
            this->feedbackFactorS3 = -1.0 / (1.0 + g);
            
            this->gainFactor = 1.0 / (1.0 - resonance * gain + resonance * gain * gain);
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
            
            if (this->resonance > 0)
                this->y *= 1.0 / this->resonance;
        }
        
        void setCoefficients(double sampleRate_Hz, double cutOff_Hz, double resonance) final
        {
            this->onePole1.setCoefficients(cutOff_Hz, sampleRate_Hz);
            
            auto g = this->onePole1.g;
            auto gain = this->onePole1.gain;
            
            this->onePole2.gain = gain;
            this->onePole3.gain = gain;
            
            this->feedbackFactorS2 = -gain / (1.0 + g);
            this->feedbackFactorS3 = 1.0 / (1.0 + g);
            
            this->gainFactor = 1.0 / (1.0 - resonance * gain + resonance * gain * gain);
        }
    };
}
