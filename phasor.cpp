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
    void Phasor::setIncrement(long double increment)
    {
        this->increment = increment;
    }
    
    void Phasor::setIncrement(unit::hertz<long double> frequency, unit::hertz<long double> sampleRate)
    {
        setIncrement(frequency.value / sampleRate.value);
    }
    
    void Phasor::doIncrement()
    {
        incrementUnwrappedPhases();
        computeNewPhases();
        recomputeGenerators();
    }
    
    long double Phasor::doIncrementAndGetPhase()
    {
        doIncrement();
        return getPhase();
    }
    
    void Phasor::setPhase(long double phase, bool recomputeGenerators)
    {
        this->phase = math::wrap<long double>(phase, 0, 1);
        
        if (recomputeGenerators)
        {
            this->recomputeGenerators();
        }
    }
    
    Phasor& Phasor::addSlave(std::unique_ptr<Phasor> slave)
    {
        if (slave->master != nullptr)
            throw std::runtime_error("slave already has a master!");
        
        slave->master = this;
        slaves.emplace_back(std::move(slave));
        return *slaves.back();
    }
    
    const Phasor* Phasor::getMaster() const
    {
        return master;
    }
    
    bool Phasor::hasMaster() const
    {
        return master != nullptr;
    }
    
    void Phasor::incrementUnwrappedPhases()
    {
        unwrappedPhase = phase + increment;
        
        for (auto& slave : slaves)
            slave->incrementUnwrappedPhases();
    }
    
    void Phasor::computeNewPhases()
    {
        phase = math::wrap<long double>(unwrappedPhase, 0, 1);
        
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
