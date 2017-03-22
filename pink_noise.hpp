//
//  PinkNoise.hpp
//  grizzly
//
//  Created by Stijn Frishert on 20/02/17.
//
//

#ifndef GRIZZLY_PINK_NOISE_HPP
#define GRIZZLY_PINK_NOISE_HPP

#include <cmath>
#include <cstddef>
#include <dsperados/math/random.hpp>
#include <dsperados/math/utility.hpp>
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
            // Create the bins
            bins.resize(order);
        }
        
        //! Generate a new pink noise sample
        template <typename Engine>
        T generate(Engine& engine)
        {
            // Generate a new white noise sample every frame
            const auto white = math::generateUniformRandom<T>(-1, 1, engine);
            if (bins.size() == 0)
                return white;
            
            // Find the correct bin we need to update
            const auto bin = std::min<float>(countTrailingZeroes(counter++), bins.size());
            
            if (bin > 0)
            {
                // Subtract the previous value, generate a new sample and add it to the running sum
                runningSum -= bins[bin];
                bins[bin] = math::generateUniformRandom<T>(-1, 1, engine);
                runningSum += bins[bin];
            }
            
            return (white + runningSum) / bins.size();
        }
        
        //! Change the order of the pink noise
        /*! The higher the order, the better the quality */
        void setOrder(std::size_t order)
        {
            bins.resize(order);
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
    };
}

#endif /* GRIZZLY_PINK_NOISE_HPP */
