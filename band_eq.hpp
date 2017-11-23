//
//  band_eq.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 22/11/2017.
//

#include <functional>
#include <stdexcept>
#include <vector>

#include "biquad.hpp"

namespace dsp
{
    template <typename T>
    class BandEQ
    {
    public:
        class Band
        {
        public:
            using CookingMethod = std::function<void(BiquadCoefficients<T>&, float, float, float, float)>;

            struct Parameters
            {
                float cutOff_Hz = 0;
                float q = 0.5;
                float gain_dB = 0;
            };
            
        public:
            template <typename Function>
            Band(float sampleRate_Hz, float cutOff_Hz, float q, float gain_dB, Function cookingMethod) :
            sampleRate_Hz(sampleRate_Hz)
            {
                parameters = {cutOff_Hz, q, gain_dB};
                setCookingMethod(cookingMethod);
            }
            
            T process(T x)
            {
                return filter.writeAndRead(x);
            }
            
            void setSampleRate(float sampleRate_Hz)
            {
                this->sampleRate_Hz = sampleRate_Hz;
                cook();
            }
            
            void setParameters(float cutOff_Hz, float q, float gain_dB)
            {
                parameters = {cutOff_Hz, q, gain_dB};
                cook();
            }
            
            void setCookingMethod(CookingMethod cookingMethod)
            {
                this->cookingMethod = cookingMethod;
                cook();
            }
            
            void setCookingMethod(void (*cookingMethod)(BiquadCoefficients<T>&, float, float, float, float))
            {
                this->cookingMethod = [=](BiquadCoefficients<T>& coefficients, float sampleRate_Hz, float cutOff_Hz, float q, float gain_dB)
                {
                    cookingMethod(coefficients, sampleRate_Hz, cutOff_Hz, q, gain_dB);
                };
                
                cook();
            }
            
            void setCookingMethod(void (*cookingMethod)(BiquadCoefficients<T>&, float, float, float))
            {
                this->cookingMethod = [=](BiquadCoefficients<T>& coefficients, float sampleRate_Hz, float cutOff_Hz, float q, float /* no gain */)
                {
                    cookingMethod(coefficients, sampleRate_Hz, cutOff_Hz, q);
                };
                
                cook();
            }
            
            void setCoefficients(BiquadCoefficients<T>& coefficients)
            {
                filter.coefficients = coefficients;
                cook();
            }
            
            void copySettings(const Band& band)
            {
                filter.coefficients = band.filter.coefficients;
                parameters = band.parameters;
                cookingMethod = band.cookingMethod;
            }
            
        private:
            void cook()
            {
                if (cookingMethod)
                    cookingMethod(filter.coefficients, sampleRate_Hz, parameters.cutOff_Hz, parameters.q, parameters.gain_dB);
            }
            
        private:
            float sampleRate_Hz = 0;
            
            CookingMethod cookingMethod;
            
            BiquadDirectForm1<T> filter;
            
            Parameters parameters;
        };
    public:
        BandEQ() = default;
        
        T process(T x)
        {
            T y = x;
            for (auto& band : bands)
                y = band.process(y);
            
            return y;
        }
        
        void setSampleRate(float sampleRate_Hz)
        {
            for (auto& band : bands)
                band.setSampleRate(sampleRate_Hz);
        }
        
        void copySettings(BandEQ& eq)
        {
            if (bands.size() != eq.bands.size())
                throw std::runtime_error("Number of bands do not match");
            
            for (auto i = 0; i < bands.size(); i++)
                bands[i].copySettings(bands[i]);
        }
        
        size_t size() const { return bands.size(); }
        
        Band& operator[](size_t i) { return bands[i]; }
        const Band& operator[](size_t i) const { return bands[i]; }
        
        auto begin() { return bands.begin(); }
        const auto begin() const { return bands.begin(); }
        
        auto end() { return bands.end(); }
        const auto end() const { return bands.end(); }
        
    public:
        std::vector<Band> bands;
    };
}
