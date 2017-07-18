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
#include <vector>

#include "delay.hpp"
#include "fast_fourier_transform.hpp"

namespace dsp
{

    class ConvolutionFFT
    {
    public:
        ConvolutionFFT(size_t frameSize, std::vector<float> kernel) :
        frameSize(frameSize),
        doubleFrameSize(2 * frameSize),
        fft(doubleFrameSize),
        delay(0),
        olaBuffer(frameSize)
        {
            size_t numberOfKernelFrames = kernel.size() / frameSize + 1;
            
            // Resize to fit last frame with frame size
            kernel.resize(frameSize * numberOfKernelFrames);
            
            delay.setMaximalDelayTime(numberOfKernelFrames - 1);
            
            resultMatrix.resize(numberOfKernelFrames, std::vector<std::complex<float>>(fft.realSpectrumSize));
            
            // fill kernel
            std::vector<std::vector<float>> kernelInZeroPaddedFrames(numberOfKernelFrames, std::vector<float>(doubleFrameSize));
            for (auto i = 0; i < numberOfKernelFrames; i++)
            {
                std::copy(kernel.begin() + frameSize * i, kernel.begin() + frameSize * (i + 1), kernelInZeroPaddedFrames[i].begin());
                fftKernel.emplace_back(fft.forward(kernelInZeroPaddedFrames[i].data()));
                delay.write(std::vector<std::complex<float>>(fft.realSpectrumSize));
            }
        }
        
        std::vector<float> process(std::vector<float> x)
        {
            // Resize to prevent circular convolution
            x.resize(doubleFrameSize);
            
            // Take fft of x
            delay.write(fft.forward(x.data()));
            
            // Convolve
            for (auto frame = 0; frame < fftKernel.size(); frame++)
                for (auto i = 0; i < fftKernel[frame].size(); i++)
                    resultMatrix[frame][i] = fftKernel[frame][i] * delay.read(frame)[i];
            
            // Initialse the output with the ola buffer
            std::vector<float> y = olaBuffer;
            
            // Reset ola buffer with zeros
            std::fill(olaBuffer.begin(), olaBuffer.end(), 0);
            
            for (auto frame = 0; frame < resultMatrix.size(); frame++)
            {
                auto inv = fft.inverse(resultMatrix[frame]);
                for (auto i = 0; i < frameSize; i ++)
                {
                    y[i] += inv[i];
                    olaBuffer[i] += inv[i + frameSize];
                }
            }
            
            return y;
        }
        
    private:
        size_t frameSize = 0;
        
        size_t doubleFrameSize = 0;
        
        dsp::FastFourierTransform fft;
        
        dsp::Delay<std::vector<std::complex<float>>> delay;
        
        std::vector<float> olaBuffer;
        
        std::vector<std::vector<std::complex<float>>> resultMatrix;
        
        std::vector<std::vector<std::complex<float>>> fftKernel;
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
        
        std::vector<float> output(outputSize);
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
