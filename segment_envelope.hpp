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
 along with this program. If not, see <http://www.gnu.org/licenses/>
 
 --------------------------------------------------------------------
 
 If you would like to use Grizzly for commercial or closed-source
 purposes, please contact us for a commercial license.
 
 */

#ifndef GRIZZLY_SEGMENT_ENVELOPE_HPP
#define GRIZZLY_SEGMENT_ENVELOPE_HPP

#include <cassert>
#include <cstddef>
#include <functional>
#include <initializer_list>
#include <mutex>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include <moditone/math/ease.hpp>
#include <moditone/math/interpolation.hpp>


namespace dsp
{
    //! Segment Envelope
    /*! Envelope generator using sequential segments */
    template <typename Value, typename Time = float>
    class SegmentEnvelope
    {
    public:
        //! Segment class which describes the individual paths of the envelope
        class Segment
        {
        public:
            //! Construct a segment
            /*! @param amplitude The amplitude value, starting from the pervious amplitude or zero at envelope start
             @param duration The duration in seconds to get to the destination amplitude.
             @param ease An easing function to alther the shape of the segment */
            Segment(SegmentEnvelope& envelope, Value amplitude, Time duration, std::function<double(double)> ease = nullptr);
            
            //! Set the duration
            /*! Any negative duration will be clamped to 0 */
            void setDuration(Time duration);
            
            //! Get the duration
            const Time& getDuration() const { return duration; }
            
            //! Retrieve the absolute time of the segment
            Time getAbsoluteTime() const;
            
            //! Given a time within the segment and its starting value, compute an interpolated value
            Value interpolate(Time time, const Value& startValue);
            
        public:
            //! Destination amplitude
            Value amplitude = 0;
            
            //! Ease function to shape the segment
            std::function<double(double)> ease = [](auto x){ return math::easeLinear(x); };
            
        private:
            //! Return the eased proportion along the time axis of this axis
            double computeTimeProportion(Time time);
            
        private:
            //! The envelope that owns this segment
            SegmentEnvelope& envelope;
            
            //! Duration
            Time duration = 0;
        };
        
    public:
        //! Construct an envelope without any segments
        SegmentEnvelope() = default;
        
        //! Construct an envelope with segments
        SegmentEnvelope(std::initializer_list<std::tuple<Value, Time>> segments);
        
        //! Move constructor
        SegmentEnvelope(SegmentEnvelope&& rhs) { operator=(std::move(rhs)); }
        
        //! Move assignment
        SegmentEnvelope& operator=(SegmentEnvelope&& rhs);
        
        //! Increment the internal time of the envelope
        void increment(Time increment);
        
        //! Return the current envelope value
        Value read();
        
        //! Set the state of the envelope
        /*! Jump directly to a certain point, changing the current segment accordingly. */
        void setTime(Time to);
        
        //! Reset the envelope to its starting position, and enable its hold
        void reset();
        
        // --- Segment insertion/removal --- //
        
        //! Add a segment
        template <typename... Args>
        void emplace(Args&&... args);
        
        //! Insert a segment
        template <typename... Args>
        void insert(std::size_t index, Args&&... args);
        
        //! Erase a segment
        void erase(std::size_t index);
        
        //! Clear all segments
        void clear();
        
        // --- Hold point manipulation --- //
        
        //! Set a hold point (and enable it on default)
        void setAndEnableHoldPoint(Time at);
        
        //! Remove hold point
        void removeHoldPoint();
        
        //! Enable the hold point, if there is one
        void enableHold();
        
        //! Disable the hold point, if there is one
        void disableHold();
        
        //! Retrieve the hold point
        Time* getHold() const;
        
        // --- Access to the segments --- //
        
        //! Retrieve the previous segment
        const Segment* previous(const Segment& segment) const;
        
        //! Retrieve the next segment
        const Segment* next(const Segment& segment) const;
        
        //! Get a segment
        auto& operator[](size_t index) { return segments[index]; }
        
        //! Get a segment, const
        const auto& operator[](size_t index) const { return segments[index]; }
        
        // Begin and end for ranged for-loop
        auto begin() { return segments.begin(); }
        auto begin() const { return segments.begin(); }
        auto end() { return segments.end(); }
        auto end() const { return segments.end(); }
        
        // --- Utility construction functions --- //
        
        //! Create an attack, sustain, release envelope
        static SegmentEnvelope ar(Time attack, Time release, bool hold = true);
        
        //! Create an attack, decay, sustain, release envelope
        static SegmentEnvelope adsr(Time attack, Time decay, Value sustain, Time release);
        
    public:
        std::function<void()> onEnd; //! Called when the envelope reached its end
        
    private:
        //! Hold point
        /*! Contraints a time point from which the envelope stops incrementing and a boolian to indicate whether the hold is enabled or not. */
        struct Hold
        {
            Time timePoint = 0;
            bool enabled = false;
        };
        
    private:
        //! Make sure the index is at a correct segment
        void canonizeIndex();
        
    private:
        //! All the segments in the envelope (in sequential order)
        std::vector<Segment> segments;
        
        //! The segment index
        size_t index = 0;
        
        //! The current time in the current segment
        Time segmentTime = 0;
        
        //! The current time in the entire envelope
        Time envelopeTime = 0;
        
        //! Optional hold point
        /*! The envelope will remain at this point until the hold point is disable */
        std::unique_ptr<Hold> hold;
        
        //! A mutex for changing the envelope while its running
        std::mutex mutex;
    };
    
    template <typename Value, typename Time>
    SegmentEnvelope<Value, Time>::SegmentEnvelope(std::initializer_list<std::tuple<Value, Time>> segments)
    {
        for (auto& segment : segments)
            this->segments.emplace_back(*this, std::get<0>(segment), std::get<1>(segment));
    }
    
    template <typename Value, typename Time>
    SegmentEnvelope<Value, Time>& SegmentEnvelope<Value, Time>::operator=(SegmentEnvelope&& rhs)
    {
        segments.clear();
        for (auto& segment : rhs.segments)
            segments.emplace_back(*this, segment.amplitude, segment.getDuration(), segment.ease);
        rhs.segments.clear();
        
        index = std::move(rhs.index);
        segmentTime = std::move(rhs.segmentTime);
        envelopeTime = std::move(rhs.envelopeTime);
        hold = std::move(rhs.hold);
        
        return *this;
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::increment(Time increment)
    {
        std::unique_lock<std::mutex> lock(mutex);
        
        // If we've reached the last segment, bail out
        if (index >= segments.size())
            return;
        
        // Increment the overall time, and the time within the segment
        segmentTime += increment;
        envelopeTime += increment;
        
        // If we've got a hold, it's enabled, pause at the hold time
        if (hold && hold->enabled && envelopeTime >= hold->timePoint)
        {
            segmentTime -= envelopeTime - hold->timePoint;
            envelopeTime = hold->timePoint;
        }
        
        // Make sure we're at the correct index
        canonizeIndex();
    }
    
    template <typename Value, typename Time>
    Value SegmentEnvelope<Value, Time>::read()
    {
        std::unique_lock<std::mutex> lock(mutex);
        
        // Make sure we're at the correct index
        canonizeIndex();
        
        // If we've reached the last segment, return the last amplitude
        if (index >= segments.size())
            return segments.empty() ? 0 : segments.back().amplitude;
        
        return segments[index].interpolate(segmentTime, index == 0 ? Value(0) : segments[index - 1].amplitude);
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::setTime(Time to)
    {
        std::unique_lock<std::mutex> lock(mutex);
        
        if (segments.empty() || to <= 0)
        {
            envelopeTime = 0;
            segmentTime = 0;
            index = 0;
            canonizeIndex();
            return;
        }
        
        // accumulate all segments durations
        auto envelopeDuration = std::accumulate(segments.begin(), segments.end(), Time(0) , [](const auto& acc, const auto& segment) { return acc + segment.getDuration(); } );
        
        if (to >= envelopeDuration)
        {
            envelopeTime = envelopeDuration;
            segmentTime = segments.back().getDuration();
            index = segments.size();
            canonizeIndex();
            return;
        }
        
        envelopeTime = to;
        
        Time partialTime = 0;
        for (auto i = 0; i < segments.size(); ++i)
        {
            if (partialTime + segments[i].getDuration() < envelopeTime)
            {
                partialTime += segments[i].getDuration();
                continue;
            }
            
            index = i;
            segmentTime = envelopeTime - partialTime;
            break;
        }
        
        canonizeIndex();
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::reset()
    {
        setTime(0);
        enableHold();
    }
    
    template <typename Value, typename Time>
    template <typename... Args>
    void SegmentEnvelope<Value, Time>::emplace(Args&&... args)
    {
        segments.emplace_back(*this, std::forward<Args&&>(args)...);
    }
    
    template <typename Value, typename Time>
    template <typename... Args>
    void SegmentEnvelope<Value, Time>::insert(std::size_t index, Args&&... args)
    {
        segments.insert(segments.begin() + index, *this, std::forward<Args&&>(args)...);
        setTime(envelopeTime);
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::erase(std::size_t index)
    {
        if (segments.empty() || index >= segments.size())
            return;
        
        segments.erase(segments.begin() + index);
        setTime(envelopeTime);
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::clear()
    {
        segments.clear();
        index = 0;
        segmentTime = 0;
        envelopeTime = 0;
        hold = nullptr;
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::setAndEnableHoldPoint(Time at)
    {
        hold = std::make_unique<Hold>(Hold{at, true});
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::removeHoldPoint()
    {
        hold = nullptr;
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::enableHold()
    {
        if (hold)
            hold->enabled = true;
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::disableHold()
    {
        if (hold)
            hold->enabled = false;
    }
    
    template <typename Value, typename Time>
    Time* SegmentEnvelope<Value, Time>::getHold() const
    {
        if (hold)
            return &hold->timePoint;
        else
            return nullptr;
    }
    
    template <typename Value, typename Time>
    const typename SegmentEnvelope<Value, Time>::Segment* SegmentEnvelope<Value, Time>::previous(const Segment& segment) const
    {
        auto it = std::find_if(segments.rbegin(), segments.rend(), [&](const auto& s){ return &s == &segment; });
        if (it == segments.rend())
            return nullptr;
        
        std::advance(it, 1);
        if (it == segments.rend())
            return nullptr;
        
        return &*it;
    }
    
    template <typename Value, typename Time>
    const typename SegmentEnvelope<Value, Time>::Segment* SegmentEnvelope<Value, Time>::next(const Segment& segment) const
    {
        auto it = std::find_if(segments.begin(), segments.end(), [&](const auto& s){ return &s == &segment; });
        if (it == segments.end())
            return nullptr;
        
        std::advance(it, 1);
        if (it == segments.end())
            return nullptr;
        
        return &*it;
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::canonizeIndex()
    {
        while (index < segments.size() && segmentTime >= segments[index].getDuration())
        {
            segmentTime -= segments[index].getDuration();
            ++index;
            
            if (index == segments.size() && onEnd)
                onEnd();
        }
        
        if (index < segments.size())
            assert(segments[index].getDuration() != 0);
    }
    
    // --- Static constructors --- //
    
    template <typename Value, typename Time>
    SegmentEnvelope<Value, Time> SegmentEnvelope<Value, Time>::ar(Time attack, Time release, bool hold)
    {
        SegmentEnvelope<Value, Time> env = {{1, attack}, {0, release}};
        if (hold)
            env.setAndEnableHoldPoint(attack);
        return env;
    }
    
    template <typename Value, typename Time>
    SegmentEnvelope<Value, Time> SegmentEnvelope<Value, Time>::adsr(Time attack, Time decay, Value sustain, Time release)
    {
        SegmentEnvelope<Value, Time> env = {{1, attack}, {sustain, decay}, {0, release}};
        env.setAndEnableHoldPoint(attack + decay);
        return env;
    }
    
    // --- Segment --- //
    
    template <typename Value, typename Time>
    SegmentEnvelope<Value, Time>::Segment::Segment(SegmentEnvelope& envelope, Value amplitude, Time duration, std::function<double(double)> ease) :
        amplitude(amplitude),
        ease(ease),
        envelope(envelope),
        duration(duration)
    {
        
    }
    
    template <typename Value, typename Time>
    void SegmentEnvelope<Value, Time>::Segment::setDuration(Time duration)
    {
        std::unique_lock<std::mutex> lock(envelope.mutex);
        this->duration = duration < 0 ? 0 : duration;
    }
    
    template <typename Value, typename Time>
    Value SegmentEnvelope<Value, Time>::Segment::interpolate(Time time, const Value& startValue)
    {
        return math::interpolateLinear(computeTimeProportion(time), startValue, amplitude);
    }
    
    template <typename Value, typename Time>
    double SegmentEnvelope<Value, Time>::Segment::computeTimeProportion(Time time)
    {
        // This function should never be called with a duration of 0
        assert(duration != 0);
        
        double x = time / static_cast<double>(duration);
        return ease ? ease(x) : x;
    }
    
    template <typename Value, typename Time>
    Time SegmentEnvelope<Value, Time>::Segment::getAbsoluteTime() const
    {
        auto prev = envelope.previous(*this);
        return prev ? (prev->getAbsoluteTime() + prev->getDuration()) : 0;
    }
}


#endif
