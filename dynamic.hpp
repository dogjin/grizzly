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

#ifndef GRIZZLY_DYNAMIC_HPP
#define GRIZZLY_DYNAMIC_HPP

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>


namespace dsp
{
    //! Compressor make-up gain (in dB) estimation for a 'standard' mixing situation
    template <class T>
    T computeCompressorMakeUpGain(T threshold_dB, T ratio)
    {
        if (ratio <= 0)
            throw std::invalid_argument("ratio <= zero");
        
        return 0.5 * threshold_dB * (1 / ratio - 1);
    }
    
    //! Dynamic Gain
    /*! Compute a gain factor for signals below/above a given threshold (dB) or within a knee range.
     A typical application would a compressor or expander.
     For this application we need to compute the gain factor from a followed
     input given a threshold, both in the log domain (e.g. deciBels).
     A gain factor, in log domain, is returned.
     This factor can be added to the original signal is it's also in log domain.
     More common however is to to convert the factor to the
     linear domain and multiply it with the orignal linear signal. */
    template <class T>
    class DynamicGain
    {
    public:
        //! Compute a gain factor (dB) for signals exceeding a threshold (dB)
        T computeGainAbove(T input_dB, T threshold_dB) const
        {
            // Compute gain factor above the threshold and knee range
            if (input_dB > threshold_dB + halfKnee_dB)
                return -slope * (input_dB - threshold_dB);
            
            // Throughput, no gain
            if (input_dB <= threshold_dB - halfKnee_dB)
                return 0;
            
            // One of the two above if statements should fire if the knee is 0, so we should never reach this
            assert(knee_dB != 0);
            
            // Compute gain factor within the knee range
            return -slope * std::pow(input_dB - threshold_dB + halfKnee_dB, 2) * doubleKneeReciprocal;
        }
        
        //! Compute a gain factor (dB) for signals below a threshold (dB)
        T computeGainBelow(T input_dB, T threshold_dB) const
        {
            // Throughput, no gain
            if (input_dB >= threshold_dB + halfKnee_dB)
                return 0;
            
            //! Compute gain factor below the threshold and knee range
            if (input_dB < threshold_dB - halfKnee_dB)
                return -slope * (input_dB - threshold_dB);
            
            // One of the two above if statements should fire if the knee is 0, so we should never reach this
            assert(knee_dB != 0);
            
            //! Apply compression within knee range
            return slope * std::pow(threshold_dB - input_dB + halfKnee_dB, 2) * doubleKneeReciprocal;
        }
        
        //! Set the ratio
        void setRatio(T ratio)
        {
            if (ratio <= 0)
                throw std::invalid_argument("ratio <= zero");
            
            slope = 1 - 1 / ratio;
        }
        
        //! The the knee range (dB) around the threshold (dB)
        void setKnee(T knee_dB)
        {
            this->knee_dB = knee_dB;
            halfKnee_dB = knee_dB * 0.5;
            doubleKneeReciprocal = 1 / (knee_dB * 2);
        }
        
    public:
        //! The slope factor
        T slope = 0;
        
    private:
        //! The knee range (dB)
        T knee_dB = 0;
        
        //! Half-knee (dB)
        T halfKnee_dB = 0;
        
        //! Double-knee reciprocal (dB)
        T doubleKneeReciprocal = 0;
    };
}

#endif
