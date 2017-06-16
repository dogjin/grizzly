//
//  HardClip.hpp
//  BandLimitedHardClip
//
//  Created by Milan van der Meer on 12/06/2017.
//
//

#ifndef GRIZZLY_CLIP_HPP
#define GRIZZLY_CLIP_HPP

#include <cmath>

namespace dsp
{
    template <typename T>
    T hardClip(const T& x, const T& threshold)
    {
        if (std::abs(x) > threshold)
            return std::signbit(x) ? -threshold : threshold;
        else
            return x;
    }
    
    
    template <class T>
    class HardClipAntiAliased
    {
    public:
        T process(const T& x, const T& threshold)
        {
            T y = 0;
            
            // Implement clipping and detect clipping points (corners)
            if (std::abs(x) >= threshold)
            {
                clipState = true;
                y = signbit(x) ? -threshold : threshold;
            }
            else
            {
                clipState = false;
                y = x;
            }
            
            // If clipping point then implement correction
            if (clipState != previousClipState)
            {
                float direction = signbit(xz1) ? -1 : 1;
                float m = x - xz1;
                float d = (direction * threshold - xz1) / m;
                
                float p1 = -(d*d*d)/6.0 + (d*d)/2.0 - d/2.0 + 1.0/6.0;
                float p0 = (d*d*d)/6.0;
                
                yz1 -= direction * std::abs(m) * p1;
                y -= direction * std::abs(m) * p0;
            }
            
            T out = yz1;
            
            yz1 = y;
            xz1 = x;
            previousClipState = clipState;
            
            return out;
        }
        
    private:
        T yz1 = 0;
        T xz1 = 0;
        
        bool clipState = false;
        bool previousClipState = false;
    };
}


#endif
