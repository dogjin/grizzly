//
//  wavetable.hpp
//  Grizzly
//
//  Created by Stijn Frishert on 11/7/17.
//

#ifndef GRIZZLY_WAVETABLE_HPP
#define GRIZZLY_WAVETABLE_HPP

namespace dsp
{
    template <typename T>
    class Wavetable
    {
    public:
        Wavetable(std::size_t size) :
            data(size)
        {
            
        }
        
        T& operator[](std::size_t index) { return data[index]; }
        const T& operator[](std::size_t index) const { return data[index]; }
        
        T get(long double phase) const
        {
            return data[static_cast<std::size_t>(phase * data.size()) % data.size()];
        }
        
        template <typename Function>
        void fill(Function func)
        {
            if (data.empty())
                return;
            
            for (auto i = 0; i < data.size(); ++i)
                data[i] = func(i / static_cast<long double>(data.size()));
        }
        
    private:
        std::vector<T> data;
    };
}

#endif /* GRIZZLY_WAVETABLE_HPP */
