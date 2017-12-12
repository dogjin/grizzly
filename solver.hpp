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
    std::unique_ptr<float> findRoot(const std::function<T(T)> function,
                                    const std::function<T(T)> derivative,
                                    T estimate, const float error, const size_t maxIterations)
    {
        for (size_t i = 0; i < maxIterations; i++)
        {
            // update result
            auto result = estimate - function(estimate) / derivative(estimate);
            
            auto residu = std::abs(result - estimate);
            // check if the residu is below the error
            if (residu < error)
                return std::make_unique<float>(result);
            
            // update estimate with current result
            estimate = result;
        }
        
        // no root was found
        return nullptr;
    }
    
    template <typename T>
    T solveImplicit(const std::function<T(T)> function,
                    const std::function<T(T)> derivative,
                    T yEstimate, const float error, const size_t maxIterations)
    {
        for (auto i = 0; i < maxIterations; i++)
        {
            // compute the y based on the function that also uses the estimate
            T y = function(yEstimate);
            
            // get the resiude
            auto residue = y - yEstimate;
            
            // We want y and the y estimate to converge,
            // if the residue is smaller than the error, the estimate is close enough
            if (std::abs(residue) <= error)
                return yEstimate;
            
            // if not, compute a new estimate
            /* this will update the outputEstimate outside the function
             * and is used in the next iteration in the function and derivative. */
            yEstimate = yEstimate - residue / derivative(yEstimate);
        }
        
        return yEstimate;
    }
}
