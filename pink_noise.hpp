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
 along with this program. If not, see <http://www.gnu.org/licenses/>
 
 --------------------------------------------------------------------
 
 If you would like to use Grizzly for commercial or closed-source
 purposes, please contact us for a commercial license.
 
 */

#ifndef GRIZZLY_PINK_NOISE_HPP
#define GRIZZLY_PINK_NOISE_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <moditone/math/random.hpp>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace dsp
{
    //! A class for generating pink noise using the Voss-McCartney algorithm
    /*! See http://www.firstpr.com.au/dsp/pink-noise/ for more information */
    template <typename T>
    class PinkNoise
    {
    public:
        //! Construct the pink noise with a given order
        /*! The higher the order, the better the quality */
        PinkNoise(std::size_t order)
        {
            setOrder(order);
        }
        
        //! Generate a new pink noise sample
        template <typename Engine>
        T generate(Engine& engine)
        {
            // Generate a new white noise sample every frame
            auto white = math::generateUniformRandom<T>(-1, 1, engine);
            if (bins.size() == 0)
                return white;
            
            white *= factor;
            
            // Find the correct bin we need to update
            const auto bin = std::min<float>(countTrailingZeroes(counter++), bins.size() - 1);
            
            if (bin > 0)
            {
                // Subtract the previous value, generate a new sample and add it to the running sum
                runningSum -= bins[bin];
                bins[bin] = math::generateUniformRandom<T>(-1, 1, engine) * factor;
                runningSum += bins[bin];
            }
            
            return (white + runningSum) / bins.size();
        }
        
        //! Change the order of the pink noise
        /*! The higher the order, the better the quality */
        void setOrder(std::size_t order)
        {
            bins.resize(order);
            factor = T(1) / (order + 1);
            
            runningSum = std::accumulate(bins.begin(), bins.end(), T(0));
        }
        
        //! Return the order of the pink noise
        /*! The higher the order, the better the quality */
        std::size_t getOrder() const { return bins.size(); }
        
    private:
        //! Retrieve the current bin we're at
        unsigned short countTrailingZeroes(std::size_t num)
        {
            // Count the trailing zeroes
            int i = 0;
            while (((num >> i) & 1) == 0 && i < sizeof(num) * 8)
                ++i;
            
            return i;
        }
        
    private:
        //! The bins containing sample-and-hold white noise
        std::vector<T> bins;
        
        //! The running sum of all accumulated bins
        T runningSum;
        
        //! A counter to drive the bin selection algorithm
        std::uint64_t counter = 1;
        
        //! The factor to multiply with every octave
        T factor = 0;
    };
}

#endif /* GRIZZLY_PINK_NOISE_HPP */
