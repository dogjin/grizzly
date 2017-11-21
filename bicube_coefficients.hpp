//
//  bicube_coefficients.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 21/11/2017.
//

#pragma once

namespace dsp
{
    template <class T>
    struct BicubeCoefficients
    {
        //! The a0 feed-forward coefficient (gain)
        T a0 = 0;
        
        //! The a1 feed-forward coefficient
        T a1 = 0;
        
        //! The a2 feed-forward coefficient
        T a2 = 0;
        
        //! The a3 feed-forward coefficient
        T a3 = 0;
        
        //! The b1 feed-back coefficient
        T b1 = 0;
        
        //! The b2 feed-back coefficient
        T b2 = 0;
        
        //! The b3 feed-back coefficient
        T b3 = 0;
    };
    
    template <class T>
    constexpr BicubeCoefficients<T> PINK_NOISE_COEFFICIENTS = { 0.049922035, -0.095993537, 0.050612699, -0.004408786, -2.494956002, 2.017265875, -0.522189400 };
}
