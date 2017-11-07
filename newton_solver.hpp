//
//  newton_solver.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 02/10/2017.
//

#pragma once

#include <cmath>
#include <functional>
#include <memory>

namespace dsp
{
    //! Newton solver
    /*! Root finding algorithm via newton's method. Given a function and its derivative,
     the position on the x-axis of the root is returned. A nullptr is returned if no root can be found */
    template <typename T>
    std::unique_ptr<float> newtonSolver(const std::function<T(T)> function,
                                        const std::function<T(T)> derivative,
                                        T estimate, const float tolerance, const size_t maxIterations)
    {
        T result = estimate;
        
        for (size_t i = 0; i < maxIterations; i++)
        {
            // update result
            result -= function(estimate) / derivative(estimate);
            
            // check if the result is close enough
            if (std::abs(result - estimate) < std::abs(result) * tolerance)
                return std::make_unique<float>(result);
            
            // update estimate with current result
            estimate = result;
        }
        
        // no root was found
        return nullptr;
    }
}
