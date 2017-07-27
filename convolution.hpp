/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2016-2017 Dsperados <info@dsperados.com>
 
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

#ifndef GRIZZLY_CONVOLUTION_HPP
#define GRIZZLY_CONVOLUTION_HPP

#include <algorithm>
#include <complex>
#include <initializer_list>
#include <iterator>
#include <vector>
#include <utility>

#include "delay.hpp"
#include "fast_fourier_transform.hpp"

namespace dsp
{
    template <class T>
    class ConvolutionFFT
    {
    public:
        template <typename Iterator>
        ConvolutionFFT(std::size_t frameSize, Iterator kernelBegin, Iterator kernelEnd) :
            frameSize(frameSize),
            doubleFrameSize(2 * frameSize),
            fft(doubleFrameSize),
            delay(0),
            inputFftFrame(doubleFrameSize),
            outputFftFrame(doubleFrameSize),
            output(frameSize),
            olaBuffer(frameSize)
        {
            if (frameSize == 0)
                throw std::invalid_argument("convolution can't be created with a frame size of 0");
            
            const auto kernelSize = std::distance(kernelBegin, kernelEnd);
            const auto numberOfKernelFrames = kernelSize / frameSize + 1;
            
            fftKernel.reserve(numberOfKernelFrames);
            delay.setMaximalDelayTime(numberOfKernelFrames - 1);
            resultMatrix.resize(numberOfKernelFrames, fft.realSpectrumSize);
            
            // Fill fftKernel
            auto frameBegin = kernelBegin;
            while (std::distance(frameBegin, kernelEnd) >= frameSize)
            {
                // Set frameEnd next to frameBegin with frameSize as distance
                auto frameEnd = std::next(frameBegin, frameSize);
                
                // Init a frame with the data from frameBegin to frameEnd
                std::vector<T> frame(frameBegin, frameEnd);
                
                // Zero padd to doubble the frame size to avoid circular convolution
                frame.resize(doubleFrameSize, 0);
                
                // Take fft
                ComplexList list(fft.realSpectrumSize);
                fft.forward(frame.data(), list.real.data(), list.imaginary.data());
                
                // Place the result of the fft in the fftKernel
                fftKernel.emplace_back(std::move(list));
                
                // Need this to init the size of the delay element. Kan anders right?
                delay.write(fft.realSpectrumSize);
                
                // Set frameBegin to be the frameEnd for next iteration
                frameBegin = frameEnd;
            }
            
            // Init a last frame with the left-over samples to kernelEnd
            std::vector<float> lastFrame(frameBegin, kernelEnd);
            if (!lastFrame.empty())
            {
                // Zero padd to dubble the frame size to avoid circular convolution
                lastFrame.resize(doubleFrameSize, 0);
                
                // Take fft
                ComplexList list(fft.realSpectrumSize);
                fft.forward(lastFrame.data(), list.real.data(), list.imaginary.data());
                
                // Place the result of the fft in the fftKernel
                fftKernel.emplace_back(std::move(list));
                
                // Need this to init the size of the delay element. Kan anders right?
                delay.write(fft.realSpectrumSize);
            }
        }
        
        template <typename Iterator>
        std::vector<T> process(Iterator frameBegin, Iterator frameEnd)
        {
            std::copy(frameBegin, frameEnd, inputFftFrame.begin());
            
            // Take fft of x
            ComplexList list(fft.realSpectrumSize);
            fft.forward(inputFftFrame.data(), list.real.data(), list.imaginary.data());
            
            delay.write(std::move(list));
            
            // Convolve
            for (auto frame = 0; frame < fftKernel.size(); frame++)
            {
                for (auto i = 0; i < fft.realSpectrumSize; i++)
                {
                    auto& x = fftKernel[frame].real[i];
                    auto& yi = fftKernel[frame].imaginary[i];
                    auto& u = delay.read(frame).real[i];
                    auto& vi = delay.read(frame).imaginary[i];
                    
                    resultMatrix[frame].real[i] = x * u - yi * vi;
                    resultMatrix[frame].imaginary[i] = x * vi + yi * u;
                }
            }
            
            // Set the output with the ola buffer
            output = olaBuffer;
            
            // Reset ola buffer with zeros
            std::fill(olaBuffer.begin(), olaBuffer.end(), 0);
            
            for (auto frame = 0; frame < resultMatrix.size(); frame++)
            {
                
                fft.inverse(resultMatrix[frame].real.data(), resultMatrix[frame].imaginary.data(), outputFftFrame.data());
                std::transform(outputFftFrame.begin(), outputFftFrame.begin() + frameSize, output.begin(), output.begin(), std::plus<>());
                std::transform(outputFftFrame.begin() + frameSize, outputFftFrame.end() + frameSize, olaBuffer.begin(), olaBuffer.begin(), std::plus<>());
            }
            
            return output;
        }
        
    public:
        const std::size_t frameSize = 0;
        const std::size_t doubleFrameSize = 0;
        
    private:
        struct ComplexList
        {
        public:
            ComplexList() = default;
            ComplexList(std::size_t size) : real(size), imaginary(size) { }
            
        public:
            std::vector<T> real;
            std::vector<T> imaginary;
        };
        
    private:
        dsp::FastFourierTransform fft; // double frame size
        
        dsp::Delay<ComplexList> delay;
        std::vector<ComplexList> resultMatrix;
        std::vector<ComplexList> fftKernel;
        
        std::vector<T> inputFftFrame; // double frame size
        std::vector<T> outputFftFrame; // double frame size
        std::vector<T> output; // frame size
        std::vector<T> olaBuffer; // frame size
    };
    
    
    //! Convolution, in the mathematical sense
    template <class T>
    class Convolution
    {
    public:
        //! Construct with a kernel
        Convolution(std::initializer_list<T> kernel) :
            Convolution(kernel.begin(), kernel.end())
        {
            
        }
        
        //! Construct with a kernel
        template <typename Iterator>
        Convolution(Iterator begin, Iterator end) :
        delay(std::distance(begin, end)),
        kernel(begin, end)
        {
            
        }
        
        //! Process a single sample
        T process(const T& x)
        {
            // Write the input into the delay line
            delay.write(x);
            
            // Convolve the past N samples with the kernel and sum them
            T sum = 0;
            for (auto h = 0; h < kernel.size(); ++h)
                sum += kernel[h] * delay.read(h);
            
            return sum;
        }
        
        //! Process a single sample
        T operator()(const T& x)
        {
            return process(x);
        }
        
        //! Change the kernel
        template <typename Iterator>
        void setKernel(Iterator begin, Iterator end)
        {
            kernel.assign(begin, end);
            delay.setMaximalDelayTime(kernel.size());
        }
        
        //! Return the kernel
        const std::vector<T>& getKernel() const { return kernel; }
        
    private:
        //! Delay line used for input
        Delay<T> delay;
        
        //! The convolution kernel
        std::vector<T> kernel;
    };
    
    //! Convolve two buffers, return a buffer with size input + kernel - 1 (output-side algorithm)
    template <typename InputIterator, typename KernelIterator>
    std::vector<std::common_type_t<typename InputIterator::value_type, typename KernelIterator::value_type>> convolve(InputIterator inBegin, InputIterator inEnd, KernelIterator kernelBegin, KernelIterator kernelEnd)
    {
        auto inputSize = std::distance(inBegin, inEnd);
        auto kernelSize = std::distance(kernelBegin, kernelEnd);
        auto outputSize = inputSize + kernelSize - 1;
        
        std::vector<std::common_type_t<typename InputIterator::value_type, typename KernelIterator::value_type>> output(outputSize);
        for (auto i = 0 ; i < outputSize; i++)
            for (auto h = 0; h < kernelSize; h++)
            {
                if (i - h < 0) continue;
                else if (i - h >= inputSize) continue;
                output[i] += kernelBegin[h] * inBegin[i - h];
            }
        
        return output;
    }
}


#endif
