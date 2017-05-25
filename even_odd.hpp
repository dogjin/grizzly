//
//  even_odd.hpp
//  Grizzly
//
//  Created by Stijn on 25/05/2017.
//
//

#ifndef GRIZZLY_EVEN_ODD_HPP
#define GRIZZLY_EVEN_ODD_HPP

#include <iterator>
#include <vector>

namespace dsp
{
    //! Decompose a buffer into its even part
    template <typename Iterator>
    std::vector<typename Iterator::value_type> decomposeEven(Iterator begin, Iterator end)
    {
        using T = typename Iterator::value_type;
        std::vector<T> even(std::distance(begin, end));
        
        std::advance(end, -1);
        for (auto i = 0; i < even.size(); ++i)
        {
            even[i] = (*begin + *end) / T(2);
            std::advance(begin, 1);
            std::advance(end, -1);
        }
        
        return even;
    }
    
    //! Decompose a buffer into its odd part
    template <typename Iterator>
    std::vector<typename Iterator::value_type> decomposeOdd(Iterator begin, Iterator end)
    {
        using T = typename Iterator::value_type;
        std::vector<T> odd(std::distance(begin, end));
        
        std::advance(end, -1);
        for (auto i = 0; i < odd.size(); ++i)
        {
            odd[i] = (*begin - *end) / T(2);
            std::advance(begin, 1);
            std::advance(end, -1);
        }
        
        return odd;
    }
}

#endif /* GRIZZLY_EVEN_ODD_HPP */
