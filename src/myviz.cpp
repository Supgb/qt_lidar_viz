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

#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QComboBox>
#include <QTabWidget>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "../include/myviz.h"
#include "../include/lidar_assembler.h"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QMainWindow( parent )
{
    // Construct UI and process signals/slots.
    ros::NodeHandle nh;
    assem = new _ld_assembler(nh);
    ui_constructor();


    // Next we initialize the main RViz classes.
    //
    // The VisualizationManager is the container for Display objects,
    // holds the main Ogre scene, holds the ViewController, etc.  It is
    // very central and we will probably need one in every usage of
    // librviz.
    manager_ = new rviz::VisualizationManager( render_panel_ );
    manager_->setFixedFrame("laser_link");
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    // Create a Grid display.
    grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
    ROS_ASSERT( grid_ != NULL );

    // Configure the GridDisplay the way we like it.
    grid_->subProp( "Line Style" )->setValue( "Lines" );
    grid_->subProp( "Color" )->setValue( QColor( Qt::white ) );

    // Create a Laser Scan display.
    scan_ = manager_->createDisplay("rviz/PointCloud", "point cloud", true);
    ROS_ASSERT( scan_ != NULL);
    // Configure the LaserScan display.
    scan_->subProp( "Style" )->setValue( "Flat Squares" );
    scan_->subProp( "Topic" )->setValue( "/lidar_cloud" );
    //scan_->subProp("Position Transformer")->setValue("XYZ");
    scan_->subProp( "Autocompute Intensity Bounds" )->setValue(true);


    // Create a Axes display.
    axes_ = manager_->createDisplay("rviz/Axes", "axes", true);
    ROS_ASSERT( axes_ != NULL);

}

// Destructor.
MyViz::~MyViz()
{
    delete manager_;
    delete assem;
}


/*
** Construct UI. ***RUN IN MyViz constructor.***
*/
void MyViz::ui_constructor()
{
    // Construct and lay out labels and slider controls.
    QLabel* thickness_label = new QLabel( "Line Thickness" );
    QSlider* thickness_slider = new QSlider( Qt::Horizontal );
    thickness_slider->setMinimum( 1 );
    thickness_slider->setMaximum( 100 );
    QLabel* cell_size_label = new QLabel( "Cell Size" );
    QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
    cell_size_slider->setMinimum( 1 );
    cell_size_slider->setMaximum( 100 );
    QGridLayout* controls_layout = new QGridLayout();
    controls_layout->addWidget( thickness_label, 0, 0 );
    controls_layout->addWidget( thickness_slider, 0, 1 );
    controls_layout->addWidget( cell_size_label, 1, 0 );
    controls_layout->addWidget( cell_size_slider, 1, 1 );

    // Lidar control panel.
    QTabWidget* control_tab = new QTabWidget();
    lidar_ctrl::LidarTune* tab_tune = new lidar_ctrl::LidarTune(assem);
    control_tab->addTab(tab_tune, "controls");
    lidar_ctrl::LidarInfo* tab_info = new lidar_ctrl::LidarInfo(assem);
    control_tab->addTab(tab_info, "Info");
    control_tab->setMinimumHeight(500);
    control_tab->setMaximumWidth(500);


    render_panel_ = new rviz::RenderPanel();
    render_panel_->setMinimumHeight(500);
    render_panel_->setMinimumWidth(600);

    QGridLayout* control_panel = new QGridLayout();
    control_panel->addWidget(render_panel_, 0, 0);
    control_panel->addWidget(control_tab, 0, 1, Qt::AlignTop);

    // Construct and lay out render panel.
    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->addLayout( controls_layout );
    main_layout->addLayout( control_panel );


    // Set the top-level layout for this MyViz widget.
    QWidget* main_widget = new QWidget();
    main_widget->setLayout(main_layout);
    setCentralWidget(main_widget);

    // Make signal/slot connections.
    connect( thickness_slider, SIGNAL( valueChanged( int )), this, SLOT( setThickness( int )));
    connect( cell_size_slider, SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));
    connect( assem, SIGNAL(EXIT_ROS()), this, SLOT(close()),
             Qt::QueuedConnection);

    // Widgets init.
    cell_size_slider->setValue( 1 );
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the line thickness of the grid by changing the
// grid's "Line Width" property.
void MyViz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
}














