//
//  band_eq.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 22/11/2017.
//

#include <vector>

#include "biquad.hpp"

namespace dsp
{
    template <typename T>
    class BandEQ
    {
    public:
        BandEQ(float sampleRate, float lowShelfCutOff, float highShelfCutOff) :
        sampleRate(sampleRate)
        {
            dsp::lowShelf(lowShelf.coefficients, sampleRate, lowShelfCutOff, 0.5, 0);
            dsp::highShelf(highShelf.coefficients, sampleRate, highShelfCutOff, 0.5, 0);
        }
        
        T process(T x)
        {
            T y = lowShelf.writeAndRead(x);
            
            for (auto& midBand : midBands)
                y = midBand.writeAndRead(y);
            
            return highShelf.writeAndRead(y);
        }
        
        static BandEQ<T> create3BandEQ(float sampleRate, float lowShelfCutOff, float midCutOff, float highShelfCutOff)
        {
            BandEQ<T> eq(sampleRate, lowShelfCutOff, highShelfCutOff);
            eq.midBands.emplace_back(BiquadDirectForm1<T>());
            peakConstantQ(eq.midBands.front().coefficients, sampleRate, midCutOff, 0.5, 0);
            
            return eq;
        }
        
        void setLowShelf(float cutOff_Hz, float q, float gain_dB)
        {
            dsp::lowShelf(lowShelf.coefficients, sampleRate, cutOff_Hz, q, gain_dB);
        }
        
        void setMidBand(size_t index, float cutOff_Hz, float q, float gain_dB)
        {
            dsp::peakConstantQ(midBands[index].coefficients, sampleRate, cutOff_Hz, q, gain_dB);
        }
        
        void setHighShelf(float cutOff_Hz, float q, float gain_dB)
        {
            dsp::highShelf(highShelf.coefficients, sampleRate, cutOff_Hz, q, gain_dB);
        }
        
    public:
        BiquadDirectForm1<T> lowShelf;
        std::vector<BiquadDirectForm1<T>> midBands;
        BiquadDirectForm1<T> highShelf;
        
    private:
        float sampleRate = 0;
    };
}
