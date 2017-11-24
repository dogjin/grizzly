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
    /*! @brief Multi-Band Filter
     *
     *  @disussion Create a serie of Biquad filters. Emplace each
     *  filter in a vector given a sample-rate, cut-off, q, gain and
     *  a cooking method (low-pass, band-pass ect.).
     *  The cooking method is a std::function of type void with
     *  BiquadCoefficients and four floats as arguments. See
     *  the different set methods for cooking in the Band class. The
     *  incomming signal is filtered form left to right band.
     *
     *  @see MultiCrossoverFilter
     */
    template <typename T>
    class MultiBandFilter
    {
    public:
        //! The individual filter band with its own settings and filter type
        class Band
        {
        public:
            //! The cooking method
            using CookingMethod = std::function<void(BiquadCoefficients<T>&, float, float, float, float)>;

            //! A strcut to hold the filter's parameters
            struct Parameters
            {
                float cutOff_Hz = 0;
                float q = 0;
                float gain_dB = 0;
            };
            
        public:
            //! Construct a band given a sample-rate, cut-off, q, gain and cooking method
            /*! See the setCookingMethod() functions on how to set the filter type. */
            template <typename Function>
            Band(float sampleRate_Hz, float cutOff_Hz, float q, float gain_dB, Function cookingMethod) :
            sampleRate_Hz(sampleRate_Hz)
            {
                parameters = {cutOff_Hz, q, gain_dB};
                setCookingMethod(cookingMethod);
            }
            
            //! Filter each sample
            T process(T x)
            {
                return filter.writeAndRead(x);
            }
            
            //! Set the sample-rate and re-cook the coefficients
            void setSampleRate(float sampleRate_Hz)
            {
                this->sampleRate_Hz = sampleRate_Hz;
                cook();
            }
            
            //! Set the parameters and re-cook the coefficients
            void setParameters(float cutOff_Hz, float q, float gain_dB)
            {
                if (parameters.cutOff_Hz == cutOff_Hz && parameters.q == q && parameters.gain_dB == gain_dB)
                    return;
                
                parameters = {cutOff_Hz, q, gain_dB};
                cook();
            }
            
            //! Set a cooking method for the type of filtering
            /*! Supply a std::function or lambda to set the filter.
             *  Not every type uses gain, however the value is passed
             *  on to fit the signature. */
            void setCookingMethod(CookingMethod cookingMethod)
            {
                this->cookingMethod = cookingMethod;
                cook();
            }
            
            //! Set a cooking method for the type of filtering
            /*! This method can be used for free functions with the same
             *  signature as the cookingMethod (i.e. filtes types
             *  with gain). */
            void setCookingMethod(void (*cookingMethod)(BiquadCoefficients<T>&, float, float, float, float))
            {
                this->cookingMethod = [=](BiquadCoefficients<T>& coefficients, float sampleRate_Hz, float cutOff_Hz, float q, float gain_dB)
                {
                    cookingMethod(coefficients, sampleRate_Hz, cutOff_Hz, q, gain_dB);
                };
                
                cook();
            }
            
            //! Set a cooking method for the type of filtering
            /*! This method can be used for free functions with one
             *  float less in than the signature (i.e. filtes types
             *  without gain). */
            void setCookingMethod(void (*cookingMethod)(BiquadCoefficients<T>&, float, float, float))
            {
                this->cookingMethod = [=](BiquadCoefficients<T>& coefficients, float sampleRate_Hz, float cutOff_Hz, float q, float /* no gain */)
                {
                    cookingMethod(coefficients, sampleRate_Hz, cutOff_Hz, q);
                };
                
                cook();
            }
            
            //! Copy the parameters, coefficients and cooking method from another band
            /*! Copying the settings avoids re-cooking.
             *  Useful to take over the settings from a
             *  left channel band to a right one. */
            void copySettings(const Band& band)
            {
                filter.coefficients = band.filter.coefficients;
                parameters = band.parameters;
                cookingMethod = band.cookingMethod;
            }
            
        private:
            //! Cook the filter coefficients given a method
            void cook()
            {
                if (cookingMethod)
                    cookingMethod(filter.coefficients, sampleRate_Hz, parameters.cutOff_Hz, parameters.q, parameters.gain_dB);
            }
            
        private:
            //! The sampling rate
            float sampleRate_Hz = 0;
            
            //! The cooking method
            CookingMethod cookingMethod;
            
            //! The biquad filter
            BiquadDirectForm1<T> filter;
            
            //! The parameters
            Parameters parameters;
        };
        
    public:
        //! Construct a multi-band filter without bands
        /*! Acces, emplace and modify a sigle band via
         *  the vector. */
        MultiBandFilter() = default;
        
        //! Filter the input
        T process(T x)
        {
            T y = x;
            for (auto& band : bands)
                y = band.process(y);
            
            return y;
        }
        
        //! Set the sample-rate and re-cook all the coefficients
        void setSampleRate(float sampleRate_Hz)
        {
            for (auto& band : bands)
                band.setSampleRate(sampleRate_Hz);
        }
        
        //! Copy the settings from another multi-band filter
        /*! Copy the parameters, coefficients and cooking
         *  methods for each band and avoid re-cooking.
         *  Useful to copy, for example, all the settings
         *  form the left channel to the right one.
         *  Beware that the number of bands must match. */
        void copySettings(MultiBandFilter& eq)
        {
            if (bands.size() != eq.bands.size())
                throw std::runtime_error("Number of bands do not match");
            
            for (auto i = 0; i < bands.size(); i++)
                bands[i].copySettings(bands[i]);
        }
        
        //! Get the number of bands
        size_t size() const { return bands.size(); }
        
        //! Get a single band
        Band& operator[](size_t i) { return bands[i]; }
        
        //! Get a signle band, const
        const Band& operator[](size_t i) const { return bands[i]; }
        
        //! Get the begin iterator of the bands
        auto begin() { return bands.begin(); }
        
        //! Get the const begin iterator of the bands
        const auto begin() const { return bands.begin(); }
        
        //! Get the end iterator of the bands
        auto end() { return bands.end(); }
        
        //! Get the const end iterator of the bands
        const auto end() const { return bands.end(); }
        
    public:
        //! The container to hold each band
        std::vector<Band> bands;
    };
}
