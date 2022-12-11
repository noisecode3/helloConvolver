/*
 * MIT License
 *
 * Copyright (c) 2022 Martin Bångens
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <unistd.h>
#include <sys/ioctl.h>
#include "FFTConvolver.h"
#include "AudioFFT.h"
#include "AudioFile.h"

/********************************************************************
 * printHead - prints a headline
 */
void printHead(const char* headline, size_t size, int col)
{
    for (int i = 0; i < col/2-size/2; i++)
      std::cout << "_";

    std::cout << headline;

    for (int i = 0; i < col/2-size/2; i++)
      std::cout << "_";
    std::cout << "\n";
}

/********************************************************************
 * printBuffer - prints the wave curve to the terminal with stars
 *
 * −1.0 = Most to the left
 *  0.0 = Center
 *  1.0 = Most to the right of the console (Linux)
 *
 * Variables
 *
 * size = buffer size
 * col = terminal columns
 * scale = print the curve scaled up to window vertical size (or frame buffer terminal)
 */
void printBuffer(const float buff[], size_t size, int col, bool scale = 0)
{
  float scaler = 1;
  col = col - col%2;
  if (scale)
  {
    float max = fabs(buff[0]);
    for (size_t i = 1; i < size; i++)
      if (fabs(buff[i]) > max)
        max = fabs(buff[i]);
    
    scaler = (1.0 / max)*0.995;
  }

  for(unsigned int i = 0; i < size; i++)
  {
    int spaces = ceil((col/2)*(scaler*buff[i]+1)) - 1;
    for (int j = 0; j < spaces; j++)
      std::cout << ' ';

    std::cout << "*\n";
  }
}

/********************************************************************
 * getKernel - makes a convolution kernel using the inverse fft
 * In frequency domain with complex number we can specify the desired
 * magnitude (frequency volume) and phase angle (shift of the wave
 * that makes up the frequency), negative frequency gives almost the
 * same as result as positive, but it the "left side" of the circle
 *
 * This is meant for noobs, and for education. I'm also learning
 * re[i] = magnitude
 * im[i] = phase angle
 * binF = how much frequency "i" is worth, "zero = 0 here" or dc
 * sr   = sampleRate
 * col  = teminal size
 *
 */
std::vector<float> getKernel(const int sr, int col, int cutoff = 200, bool print = 1)
{
  const size_t fftSize = 1024; // Needs to be power of 2!
  const float binF = (sr/2.0)/(fftSize/2.0);
  if (cutoff > sr/2)
    cutoff = sr/2;

  std::vector<float> re(audiofft::AudioFFT::ComplexSize(fftSize));
  std::vector<float> im(audiofft::AudioFFT::ComplexSize(fftSize));
  std::vector<float> output(fftSize);

  //******************************************************************************
  //                                 The Filter
  //******************************************************************************
  int   ci(cutoff/binF);
  float cf(cutoff/binF - ci);

  //This is a brick wall low pass sinc kernel, have fun 
  for (size_t i = 0; i < fftSize/2+1; i++)
  {
    if (i <= ci)
      re[i] = 1;
    else if (i == ci+1)
      re[i] = cf;
    else
      re[i] = 0;
  }

  for (size_t i = 0; i < fftSize/2+1; i++)
  {
    im[i] = sin(-i+ci+cf)/(-i+ci+cf);
  }

  //******************************************************************************

  audiofft::AudioFFT fft;
  fft.init(1024);
  fft.ifft(output.data(), re.data(), im.data());

  if (print)
  {
    printHead("Kernel", 6, col);
    printBuffer(output.data(), fftSize/2+1, col, true ); //scaled
    printHead("Real", 4, col);
    printBuffer(re.data(), fftSize/2+1, col);
    printHead("Imaginary_", 10, col);
    printBuffer(im.data(), fftSize/2+1, col);
  }
  return output;
}

/********************************************************************
 * main - boring stuff
 *
 * This load in.wav file into memory and preforms the fftconvolver
 * Both mono and stereo wav can be used for testing
 * and the is saves it at out.wav
 */
int main(int argc, char **argv)
{
  if (argc > 2)
  {
    std::cerr << "specify only one positive integer cutoff" << std::endl;
  }

  int cutoff = 200;
  if (argc == 2)
  {
    cutoff = atoi(argv[1]);
    if (cutoff < 0 )
      std::cerr << "specify only positive integer cutoff" << std::endl;
  }

  struct winsize wSize;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &wSize);

  AudioFile<float> audioFile; 
  audioFile.load ("in.wav");

  int sampleRate = audioFile.getSampleRate();
  //int bitDepth = audioFile.getBitDepth();

  int numSamples = audioFile.getNumSamplesPerChannel();
  //float lengthInSeconds = audioFile.getLengthInSeconds();
  std::vector<float> kernel = getKernel(sampleRate, wSize.ws_col, cutoff);

  const int convolverSize = 1024;
  const int kernelSize = kernel.size()/2 + 1;
  const int fftCalls = numSamples / 1024;
  const int fftLastCall = numSamples % 1024;

  if (audioFile.isMono())
  {
    fftconvolver::FFTConvolver Convolver;
    assert (Convolver.init(convolverSize, kernel.data(), kernelSize));
    float in[convolverSize];
    float out[convolverSize];

    for (int i = 0; i < fftCalls; i++)
    {
      for (int j = 0; j < convolverSize; j++)
      {
	      in[j] = audioFile.samples[0][j + i*convolverSize];
      }
      Convolver.process(in, &audioFile.samples[0][i*convolverSize], convolverSize);
    }
    if (fftLastCall)
    {
      for (int i = 0; i < fftLastCall; i++)
      {
	      in[i] = audioFile.samples[0][i+(fftCalls*convolverSize)];
      }
      for (int i = fftLastCall; i < convolverSize; i++)
      {
	      in[i] = 0;
      }
      Convolver.process(in, out, convolverSize);
      for (int i = 0; i < fftLastCall; i++)
      {
	      audioFile.samples[0][i + fftCalls*convolverSize] = out[i];
      }
    }
  }
  else if (audioFile.isStereo())
  {
    fftconvolver::FFTConvolver Convolver1;
    fftconvolver::FFTConvolver Convolver2;
    assert (Convolver1.init(convolverSize, kernel.data(), kernelSize));
    assert (Convolver2.init(convolverSize, kernel.data(), kernelSize));
    float in1[convolverSize];
    float in2[convolverSize];
    float out1[convolverSize];
    float out2[convolverSize];

    for (int i = 0; i < fftCalls; i++)
    {
      for (int j = 0; j < convolverSize; j++)
      {
	      in1[j] = audioFile.samples[0][j + i*convolverSize];
	      in2[j] = audioFile.samples[1][j + i*convolverSize];
      }
      Convolver1.process(in1, &audioFile.samples[0][i*convolverSize], convolverSize);
      Convolver2.process(in2, &audioFile.samples[1][i*convolverSize], convolverSize);
    }
    if (fftLastCall)
    {
      for (int i = 0; i < fftLastCall; i++)
      {
	      in1[i] = audioFile.samples[0][i + fftCalls*convolverSize];
	      in2[i] = audioFile.samples[1][i + fftCalls*convolverSize];
      }
      for (int i = fftLastCall; i < convolverSize; i++)
      {
	      in1[i] = 0;
	      in2[i] = 0;
      }
      Convolver1.process(in1, out1, convolverSize);
      Convolver2.process(in2, out2, convolverSize);
      for (int i = 0; i < fftLastCall; i++)
      {
	      audioFile.samples[0][i + fftCalls*convolverSize] = out1[i];
	      audioFile.samples[1][i + fftCalls*convolverSize] = out2[i];
      }
    }
  }
  audioFile.save ("out.wav");
  return 0;
}
