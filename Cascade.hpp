/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2017 Dsperados <info@dsperados.com>
 
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

#ifndef GRIZZLY_CASCADE_HPP
#define GRIZZLY_CASCADE_HPP

#include <functional>
#include <stdexcept>
#include <vector>

namespace dsp
{
    //! Cascade functions with a T(T) signature
    /*! A cascade can comute multiple functions in one call, using the output of one stage as the input for the next stage.
     */
    template <class T>
    class Cascade
    {
    public:
        //! Write all stages, return output
        T writeAndRead(const T& input)
        {
            write(input);
            return readOutput();
        }
        
        //! Write a value to the cascade
        void write(const T& input)
        {
            this->input = input;
            
            T stageOutput = input;
            
            for (auto i = 0; i < stages.size(); ++i)
            {
                stageOutput = stages[i](stageOutput);
                output[i] = stageOutput;
            }
        }
        
        //! Read the last stage
        T readOutput() const
        {
            if (output.empty())
                return input;
            
            return output.back();
        }
        
        //! Read the output state of a stage
        T readStage(size_t index) const
        {
            return output[index];
        }
        
        //! Read the input state
        T readInput() const
        {
            return input;
        }
        
        //! Add a stage at the back of the cascade
        void emplaceBack(std::function<T(T)> stage)
        {
            stages.emplace_back(stage);
            clearStates();
        }
        
        //! Add a stage at the back of the cascade
        /*! @param method The address of a class member function with a T(T) signature
         @param ptr The addres of your class instance containing the method */
        template <typename Method, typename This>
        void emplaceBack(Method method, This ptr, size_t position)
        {
            emplaceBack([method, ptr](T x){ return (ptr->*method)(x); });
            clearStates();
        }
        
        //! Insert a stage at a given position
        void emplace(std::function<T(T)> stage, size_t position)
        {
            if (position > stages.size())
                throw std::invalid_argument("position > stages.size()");
            
            stages.emplace(stages.begin() + position, stage);
            clearStates();
        }
        
        //! Insert a stage at a given position
        /*! @param method The address of a class member function with a T(T) signature
         @param ptr The addres of your class instance containing the method */
        template <typename Method, typename This>
        void emplace(Method method, This ptr, size_t position)
        {
            emplace([method, ptr](T x){ return (ptr->*method)(x); }, position);
            clearStates();
        }
        
        //! Erase a stage at a given position
        void erase(size_t position)
        {
            if (position >= stages.size())
                throw std::invalid_argument("position >= stages.size()");
            
            stages.erase(stages.begin() + position);
            clearStates();
        }
        
        //! Erase all stages
        void eraseAll()
        {
            stages.clear();
            clearStates();
        }
        
        //! Clear all the input and output states
        void clearStates()
        {
            output.clear();
            output.resize(stages.size());
            input = 0;
        }
        
        //! Get the number of stages
        size_t getSize() const
        {
            return stages.size();
        }
        
        //! Return a single stage
        std::function<T(T)>& operator[](size_t index) { return stages[index]; };
        
        //! Return a single stage
        const std::function<T(T)>& operator[](size_t index) const { return stages[index]; };
        
    private:
        //! The stage functions
        std::vector<std::function<T(T)>> stages;
        
        //! The output states
        std::vector<T> output;
        
        //! The input state
        T input = 0;
    };
}

#endif /* GRIZZLY_CASCADE_HPP */
