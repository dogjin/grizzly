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

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#pragma once

namespace dsp
{
    template <class T>
    class StepSequencer
    {
    private:
        class Step
        {
        public:
            Step(T time, std::function<void()> onTrigger) :
            time(time),
            onTrigger(onTrigger)
            {}
            
            void trigger()
            {
                if (onTrigger)
                    onTrigger();
            }
            
            T time = 0;
            std::function<void()> onTrigger;
        };
        
    public:
        StepSequencer(T length)
        {
            this->length = length;
        }
        
        void process(T time)
        {
            if (steps.empty())
                return;
            
            auto previousStep = currentStep;
            currentStep = getStepIndex(time);
            
            if (currentStep - previousStep == 1 || (currentStep == 0 && previousStep == steps.size() - 1))
            {
                auto& step = steps[currentStep];
                
                step.trigger();
            }
        }
        
        void operator()(T time)
        {
            process(time);
        }
        
        void emplace(T time, std::function<void()> onTrigger)
        {
            steps.emplace_back(time, onTrigger);
            std::sort(steps.begin(), steps.end(), [](const Step& lhs, const Step& rhs){ return lhs.time < rhs.time; });
        }
        
        void emplaceEqualSteps(size_t numberOfSteps)
        {
            const auto stepLength = length / numberOfSteps;
            for (auto i = 0; i < numberOfSteps; i++)
                steps.emplace_back(stepLength * i, nullptr);
        }
        
        void remove(size_t index)
        {
            steps.erase(steps.begin() + index);
        }
        
        void clear()
        {
            steps.clear();
        }
        
        void setSwing(float amount)
        {
            amount = amount * 2 - 1;
            
            const auto stepLength = length / steps.size();
            
            for (auto i = 1; i < steps.size(); i += 2)
                steps[i].time += stepLength * amount;
        }
        
        void setStep(size_t index, bool trigger)
        {
            currentStep = index;
            
            if (trigger)
                steps[currentStep].trigger();
        }
        
        Step& getStep(size_t index)
        {
            return steps[index];
        }
        
        Step& operator[](size_t index)
        {
            getStep(index);
        }
        
        const Step& getStep(size_t index) const
        {
            return steps[index];
        }
        
        const Step& operator[](size_t index) const
        {
            getStep(index);
        }
        
        auto begin() { return steps.begin(); }
        auto end() { return steps.end(); }
        
        const auto begin() const { return steps.begin(); }
        const auto end() const { return steps.end(); }
        
        T getLength() const
        {
            return length;
        }
        
        size_t getNumberOfSteps() const
        {
            return steps.size();
        }
        
    private:
        int getStepIndex(T time)
        {
            auto reverseIndex = static_cast<int>(steps.size()) - 1;
            for (; reverseIndex >= 0; reverseIndex--)
                if (steps[reverseIndex].time <= std::fmod(time, length))
                    return reverseIndex;
            
            return -1;
        }
        
    private:
        std::vector<Step> steps;
        
        //! Total cylce length
        T length = 0;
        
        int currentStep = -1;
    };
}
