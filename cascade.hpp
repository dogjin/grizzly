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

#ifndef GRIZZLY_CASCADE_HPP
#define GRIZZLY_CASCADE_HPP

#include <cstddef>
#include <functional>
#include <stdexcept>
#include <vector>

namespace dsp
{
    //! Cascade of functions with a T(T) signature
    /*! A cascade chains multiple functions with one call, using the output of one stage as the input for the next stage. */
    template <typename T>
    class Cascade
    {
    public:
        //! Input a new value into the cascade, recomputing the output of every stage
        void write(const T& input)
        {
            this->input = input;
            
            T stageOutput = input;
            for (auto& stage : stages)
                stage.output = stageOutput = stage.filter(stageOutput);
        }
        
        //! Read the input state
        T readInput() const
        {
            return input;
        }
        
        //! Read the output state of a stage
        T readStage(std::size_t index) const
        {
            if (index >= stages.size())
                throw std::out_of_range("Cascade readStage index is out of range");
            
            return stages[index].output;
        }
        
        //! Read output from the last stage
        /*! If there are not stages, the input will be returned */
        T readOutput() const
        {
            return stages.empty() ? input : stages.back().output;
        }
        
        //! Call write and then readOutput
        T writeAndReadOutput(const T& input)
        {
            write(input);
            return readOutput();
        }
        
        //! Add a filter stage to the back of the cascade
        void emplaceBack(std::function<T(const T&)> filter)
        {
            Stage stage;
            stage.filter = std::move(filter);
            stages.emplace_back(std::move(stage));
            
            clear();
        }
        
        //! Add a stage at the back of the cascade
        /*! @param method The address of a class member function with a T(T) signature
            @param ptr The addres of your class instance containing the method */
        template <typename Method, typename This>
        void emplaceBack(Method method, This ptr)
        {
            emplaceBack([method, ptr](const T& x){ return (ptr->*method)(x); });
            clear();
        }
        
        //! Insert a stage at a given position
        void emplace(std::function<T(const T&)> filter, std::size_t position)
        {
            if (position > stages.size())
                throw std::invalid_argument("Cascade position for emplace out of range");
            
            Stage stage;
            stage.filter = std::move(filter);
            stages.emplace(stages.begin() + position, std::move(stage));
            
            clear();
        }
        
        //! Insert a stage at a given position
        /*! @param method The address of a class member function with a T(T) signature
         @param ptr The addres of your class instance containing the method */
        template <typename Method, typename This>
        void emplace(Method method, This ptr, std::size_t position)
        {
            emplace([method, ptr](T x){ return (ptr->*method)(x); }, position);
            clear();
        }
        
        //! Erase a stage at a given position
        void erase(std::size_t position)
        {
            if (position >= stages.size())
                throw std::out_of_range("Cascade position for erase out of range");
            
            stages.erase(stages.begin() + position);
            clear();
        }
        
        //! Erase all stages
        void eraseAll()
        {
            stages.clear();
            clear();
        }
        
        //! Clear all the input and output states
        void clear()
        {
            input = T();
            for (auto& stage : stages)
                stage.output = T();
        }
        
        //! Get the number of stages
        std::size_t getSize() const
        {
            return stages.size();
        }
        
        //! Return a single stage
        std::function<T(const T&)>& operator[](std::size_t index)
        {
            if (index >= stages.size())
                throw std::out_of_range("Cascade position for operator[] out of range");
            
            return stages[index].filter;
        }
        
        //! Return a single stage
        const std::function<T(const T&)>& operator[](std::size_t index) const
        {
            if (index >= stages.size())
                throw std::out_of_range("Cascade position for operator[] out of range");
            
            return stages[index].filter;
        }
        
    private:
        //! Private structure containing the data for each stage
        struct Stage
        {
            std::function<T(const T&)> filter;
            T output = T{};
        };
        
    private:
        //! The stage functions
        std::vector<Stage> stages;
        
        //! The input state
        T input = 0;
    };
}

#endif /* GRIZZLY_CASCADE_HPP */
