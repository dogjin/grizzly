//
//  phasor.cpp
//  Sandbox
//
//  Created by Stijn Frishert on 20/10/2017.
//  Copyright © 2017 Stijn Frishert. All rights reserved.
//

#include <moditone/math/wrap.hpp>
#include <stdexcept>

#include "generator.hpp"
#include "phasor.hpp"

namespace dsp
{
    void Phasor::setIncrement(double increment)
    {
        this->increment = increment;
    }
    
    void Phasor::tick()
    {
        incrementUnwrappedPhases();
        computeNewPhases();
        recomputeGenerators();
    }
    
    double Phasor::tickAndGetPhase()
    {
        tick();
        return getPhase();
    }
    
    void Phasor::setPhase(double phase, bool recomputeGenerators)
    {
        this->phase = math::wrap<double>(phase, 0, 1);
        
        if (recomputeGenerators)
        {
            this->recomputeGenerators();
        }
    }
    
    void Phasor::addSlave(Phasor& slave)
    {
        if (slave.master != nullptr)
            throw std::runtime_error("slave already has a master!");
        
        slave.master = this;
        slaves.emplace_back(&slave);
    }
    
    void Phasor::incrementUnwrappedPhases()
    {
        unwrappedPhase = phase + increment;
        
        for (auto& slave : slaves)
            slave->incrementUnwrappedPhases();
    }
    
    void Phasor::computeNewPhases()
    {
        phase = math::wrap<double>(unwrappedPhase, 0.0, 1.0);
        
        if (unwrappedPhase >= 1.0l)
        {
            resetSlaves(*this);
        }
        else
        {
            for (auto& slave : slaves)
                slave->computeNewPhases();
        }
    }
    
    void Phasor::recomputeGenerators()
    {
        for (auto& generator : generators)
            generator->recompute();
        
        for (auto& slave : slaves)
            slave->recomputeGenerators();
    }
    
    void Phasor::resetSlaves(Phasor& m)
    {
        for (auto& slave : slaves)
        {
            const auto ratio = slave->getIncrement() / m.getIncrement();
            slave->setPhase(m.getPhase() * ratio, false);
            slave->resetSlaves(m);
        }
    }
}
