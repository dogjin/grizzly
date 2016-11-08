//
//  HilbertTransform.cpp
//  libbear
//
//  Created by Stijn Frishert on 02/02/16.
//  Copyright © 2016 FrisHertz. All rights reserved.
//

#include <algorithm>

#include <dsperados/math/spline.hpp>

#include "FourierTransform.hpp"
#include "HilbertTransform.hpp"
#include "Parallel.hpp"

using namespace gsl;
using namespace math;
using namespace std;

namespace bear::dsp
{
    vector<float> hilbertTransform(const vector<float>& input, bool inverse)
    {
        vector<std::complex<float>> complexInput(input.size());
        transform(input.begin(), input.end(), complexInput.begin(), [](const auto& x){ return x; });

        auto hilbert = hilbertTransformComplex(complexInput, inverse);

        vector<float> result(hilbert.size());
        transform(hilbert.begin(), hilbert.end(), result.begin(), [](const auto& x){ return x.real(); });

        return result;
    }

    vector<std::complex<float>> hilbertTransformComplex(const vector<std::complex<float>>& input, bool inverse)
    {
        // Take the forward Fourier
        auto spectrum = fourierTransformComplex(input);

        const auto halfSize = spectrum.size() / 2;

        // Multiply the first half with -j1 (or j1 for inverse)
        for (auto i = 0; i < halfSize; ++i)
            spectrum[i] *= std::complex<float>(0, inverse ? 1 : -1);

        // Multiply the second half with j1 (or -j1 for inverse)
        for (auto i = halfSize; i < spectrum.size(); ++i)
            spectrum[i] *= std::complex<float>(0, inverse ? -1 : 1);

        // Return the inverse fourier
        return inverseFourierTransformComplex(spectrum);
    }
    
    vector<float> findIntrinsicModeFunction(const vector<float>& input)
    {
        vector<float> sift(input.begin(), input.end());

        while (true)
        {
            auto minima = localMinima(const vector<float>&(sift));
            auto maxima = localMaxima(const vector<float>&(sift));
            auto crossings = zeroCrossings(const vector<float>&(sift));

            if (minima.size() + maxima.size() - (int)crossings <= 1)
                return sift;

            CubicSpline minimaSpline;
            minimaSpline.emplace<size_t>(minima, sift);

            auto minimaSignal = minimaSpline.span(0, sift.size());

            CubicSpline maximaSpline;
            maximaSpline.emplace<size_t>(maxima, sift);

            auto maximaSignal = maximaSpline.span(0, sift.size());

            auto m = mean(const vector<float>&(minimaSignal), const vector<float>&(maximaSignal));

            sift = subtract(const vector<float>&(sift), const vector<float>&(m));
        }
    }

   IntrinsicModeFunctions findIntrinsicModeFunctions(const vector<float>& input)
    {
        IntrinsicModeFunctions result;

        result.residue.resize(input.size());
        copy(input.begin(), input.end(), result.residue.begin());

        while (rootMeanSquare(const vector<float>&(result.residue)) >= 0.01)
        {
            result.intrinsicModeFunctions.emplace_back(findIntrinsicModeFunction(result.residue));
            subtract(const vector<float>&(result.residue), const vector<float>&(result.intrinsicModeFunctions.back()), vector<float>&(result.residue));
        }

        return result;
    }
}
