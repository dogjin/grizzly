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

#ifndef GRIZZLY_DYNAMIC_HPP
#define GRIZZLY_DYNAMIC_HPP

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unit/amplitude.hpp>

namespace dsp
{
    //! Compressor make-up gain estimation for a 'standard' mixing situation
    inline static unit::decibel<float> computeCompressorMakeUpGain(unit::decibel<float> threshold, float ratio)
    {
        if (ratio <= 0)
            throw std::invalid_argument("ratio <= zero");
        
        return 0.5f * threshold.value * (1.0f / ratio - 1.0f);
    }
    
    //! Dynamic Gain
    /*! Compute a gain factor for signals below/above a given threshold or within a knee range.
     A typical application is a compressor/expander where the the gain factor is computed from a followed input and multiplied with the original input. */
    class DynamicGain
    {
    public:
        //! Compute a gain factor for signals exceeding a threshold
        unit::decibel<float> computeGainAbove(unit::decibel<float> input, unit::decibel<float> threshold) const
        {
            // Compute gain factor above the threshold and knee range
            if (input.value > threshold.value + halfKnee)
                return -slope * (input.value - threshold.value);
            
            // Throughput, no gain
            if (input.value <= threshold.value - halfKnee)
                return 0;
            
            // One of the two above if statements should fire if the knee is 0, so we should never reach this
            assert(knee != 0);
            
            // Compute gain factor within the knee range
            return -slope * std::pow(input.value - threshold.value + halfKnee, 2) * doubleKneeReciprocal;
        }
        
        //! Compute a gain factor for signals below a threshold
        unit::decibel<float> computeGainBelow(unit::decibel<float> input, unit::decibel<float> threshold) const
        {
            // Throughput, no gain
            if (input.value >= threshold.value + halfKnee)
                return 0;
            
            //! Compute gain factor below the threshold and knee range
            if (input.value < threshold.value - halfKnee)
                return -slope * (input.value - threshold.value);
            
            // One of the two above if statements should fire if the knee is 0, so we should never reach this
            assert(knee != 0);
            
            //! Apply compression within knee range
            return slope * std::pow(threshold.value - input.value + halfKnee, 2) * doubleKneeReciprocal;
        }
        
        //! Set the ratio
        void setRatio(float ratio)
        {
            if (ratio <= 0)
                throw std::invalid_argument("ratio <= zero");
            
            slope = 1.f - 1.f / ratio;
        }
        
        //! The the knee range around the threshold
        void setKnee(unit::decibel<float> knee)
        {
            this->knee = knee.value;
            halfKnee = knee.value * 0.5f;
            doubleKneeReciprocal = 1.f / (knee.value * 2.f);
        }
        
    public:
        //! The slope factor
        float slope = 0;
        
    private:
        //! The knee range
        float knee = 0;
        
        //! Half-knee
        float halfKnee = 0;
        
        //! Double-knee reciprocal
        float doubleKneeReciprocal = 0;
    };
}

#endif
