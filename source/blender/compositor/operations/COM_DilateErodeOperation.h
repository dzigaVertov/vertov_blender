/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Copyright 2011, Blender Foundation.
 */

#pragma once

#include "COM_NodeOperation.h"

class DilateErodeThresholdOperation : public NodeOperation {
 private:
  /**
   * Cached reference to the inputProgram
   */
  SocketReader *m_inputProgram;

  float m_distance;
  float m__switch;
  float m_inset;

  /**
   * determines the area of interest to track pixels
   * keep this one as small as possible for speed gain.
   */
  int m_scope;

 public:
  DilateErodeThresholdOperation();

  /**
   * The inner loop of this operation.
   */
  void executePixel(float output[4], int x, int y, void *data);

  /**
   * Initialize the execution
   */
  void initExecution();

  void *initializeTileData(rcti *rect);
  /**
   * Deinitialize the execution
   */
  void deinitExecution();

  void setDistance(float distance)
  {
    this->m_distance = distance;
  }
  void setSwitch(float sw)
  {
    this->m__switch = sw;
  }
  void setInset(float inset)
  {
    this->m_inset = inset;
  }

  bool determineDependingAreaOfInterest(rcti *input,
                                        ReadBufferOperation *readOperation,
                                        rcti *output);
};

class DilateDistanceOperation : public NodeOperation {
 private:
 protected:
  /**
   * Cached reference to the inputProgram
   */
  SocketReader *m_inputProgram;
  float m_distance;
  int m_scope;

 public:
  DilateDistanceOperation();

  /**
   * The inner loop of this operation.
   */
  void executePixel(float output[4], int x, int y, void *data);

  /**
   * Initialize the execution
   */
  void initExecution();

  void *initializeTileData(rcti *rect);
  /**
   * Deinitialize the execution
   */
  void deinitExecution();

  void setDistance(float distance)
  {
    this->m_distance = distance;
  }
  bool determineDependingAreaOfInterest(rcti *input,
                                        ReadBufferOperation *readOperation,
                                        rcti *output);

  void executeOpenCL(OpenCLDevice *device,
                     MemoryBuffer *outputMemoryBuffer,
                     cl_mem clOutputBuffer,
                     MemoryBuffer **inputMemoryBuffers,
                     std::list<cl_mem> *clMemToCleanUp,
                     std::list<cl_kernel> *clKernelsToCleanUp);
};
class ErodeDistanceOperation : public DilateDistanceOperation {
 public:
  ErodeDistanceOperation();

  /**
   * The inner loop of this operation.
   */
  void executePixel(float output[4], int x, int y, void *data);

  void executeOpenCL(OpenCLDevice *device,
                     MemoryBuffer *outputMemoryBuffer,
                     cl_mem clOutputBuffer,
                     MemoryBuffer **inputMemoryBuffers,
                     std::list<cl_mem> *clMemToCleanUp,
                     std::list<cl_kernel> *clKernelsToCleanUp);
};

class DilateStepOperation : public NodeOperation {
 protected:
  /**
   * Cached reference to the inputProgram
   */
  SocketReader *m_inputProgram;

  int m_iterations;

 public:
  DilateStepOperation();

  /**
   * The inner loop of this operation.
   */
  void executePixel(float output[4], int x, int y, void *data);

  /**
   * Initialize the execution
   */
  void initExecution();

  void *initializeTileData(rcti *rect);
  /**
   * Deinitialize the execution
   */
  void deinitExecution();
  void deinitializeTileData(rcti *rect, void *data);

  void setIterations(int iterations)
  {
    this->m_iterations = iterations;
  }

  bool determineDependingAreaOfInterest(rcti *input,
                                        ReadBufferOperation *readOperation,
                                        rcti *output);
};

class ErodeStepOperation : public DilateStepOperation {
 public:
  ErodeStepOperation();

  void *initializeTileData(rcti *rect);
};
