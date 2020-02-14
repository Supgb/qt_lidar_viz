/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MYVIZ_H
#define MYVIZ_H

#include <QWidget>
#include <QMainWindow>

#include <boost/shared_ptr.hpp>
#include "../include/lidar_assembler.h"
#include "../include/lidarctrlpanel.h"
#include "../include/lidarinfo.h"
#include "../include/lidartune.h"

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "MyViz" implements the top level widget for this example.
class MyViz: public QMainWindow
{
Q_OBJECT
public:
  explicit MyViz( QWidget* parent = 0 );
  MyViz(const MyViz&) = delete;
  MyViz& operator=(const MyViz&) = delete;
  virtual ~MyViz();

  void ui_constructor();

  // MyViz pointer definition
  typedef boost::shared_ptr<MyViz> MyVizPtr;
  typedef boost::shared_ptr<const MyViz> MyVizConstPtr;
  typedef lidar_assembler::LidarAssembler::ExecMode _Mode;
  typedef lidar_assembler::LidarAssembler _ld_assembler;

private Q_SLOTS:
  // Visualization slots.
  void setThickness( int thickness_percent );
  void setCellSize( int cell_size_percent );

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* grid_;
  rviz::Display* scan_;
  rviz::Display* axes_;

  _ld_assembler *assem;

};
// END_TUTORIAL
#endif // MYVIZ_H
