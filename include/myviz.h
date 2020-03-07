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

#include <QMainWindow>

#include <boost/shared_ptr.hpp>

#include "../include/lidardriver.h"
#include "../include/lidardecoder.h"
#include "../include/lidar_assembler.h"
#include "../include/lidarmdl.h"
#include "../include/lidarctrlpanel.h"
#include "../include/lidarinfo.h"
#include "../include/lidardebug.h"

class QWidget;
class QGridLayout;
class QComboBox;
class QPushButton;
class QLineEdit;
class QSlider;
class QGroupBox;

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class MarkerDisplay;
}

namespace boost
{
class thread;
}

// BEGIN_TUTORIAL
// Class "MyViz" implements the top level widget for this example.
class MyViz final: public QMainWindow, public lidar_base::LogSys
{
Q_OBJECT
public:
    // MyViz pointer definition
    typedef boost::shared_ptr<MyViz> MyVizPtr;
    typedef boost::shared_ptr<const MyViz> MyVizConstPtr;
    typedef lidar_base::LidarAssembler::ExecMode _Mode;
    typedef lidar_base::LidarAssembler _ld_assembler;
    typedef lidar_driver::LidarDriver _ld_driver_t;
    typedef lidar_driver::LidarDecoder _ld_decoder_t;
    typedef lidar_base::LidarMDL _ld_model_t;
    typedef lidar_base::LidarDebug<_ld_model_t> _ld_debug_md_t;
    typedef lidar_base::LidarDebug<_ld_driver_t> _ld_debug_drv_t;
    typedef lidar_base::LidarDebug<_ld_decoder_t> _ld_debug_dec_t;
    typedef lidar_base::LidarDebug<_ld_assembler> _ld_debug_asb_t;

    explicit MyViz( QStringListModel* lm,
                    QWidget* parent = 0 );
    MyViz(const MyViz&) = delete;
    MyViz& operator=(const MyViz&) = delete;
    ~MyViz();

    // Log
    virtual void log_pipe(const LogLevel& level, const QString& msg)
    { log(level, msg); Q_EMIT UPDATE_LOG(); }

    // Member vars access.
    rviz::VisualizationManager* getManager()
    { return manager_; }
    rviz::MarkerDisplay* getMarker()
    { return marker_; }
    rviz::Display* getScan()
    { return scan_; }
    _ld_debug_md_t* getDebugger()
    { return _debugger_md_ptr; }
    void setBoostDebugger(boost::thread* th)
    {
        debugger_th =  th;
        log_pipe(Info, "Debug requested...");
    }

    void resetMesh();

    void ui_constructor();

Q_SIGNALS:
    void UPDATE_LOG();

private Q_SLOTS:
    // Visualization slots.
    void setThickness( int thickness_percent );
    void setCellSize( int cell_size_percent );

private:    
    ros::NodeHandle nh, ph;
    ros::AsyncSpinner spinner;

    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* grid_;
    rviz::Display* scan_;
    rviz::Display* axes_;
    rviz::MarkerDisplay* marker_;

    _ld_driver_t  *_driver_ptr;
    _ld_decoder_t *_decoder_ptr;
    _ld_assembler *assem;
    _ld_model_t *_model_ptr;
    _ld_debug_md_t *_debugger_md_ptr;

    boost::thread* debugger_th;
};

namespace lidar_base
{

class LidarTune final: public QWidget, public LidarCtrlPanel
{
    Q_OBJECT
public:
    explicit LidarTune(_ld_assembler* assem, _ld_driver_t* driver,
                       _ld_decoder_t* decoder, _ld_model_t* model, MyViz* viz);
    LidarTune(const LidarTune&) = delete;
    LidarTune& operator =(const LidarTune&) = delete;
    virtual ~LidarTune() = default;

    virtual void ui_constructor();


    bool checkValid();
    bool checkValid(double);

private Q_SLOTS:
    void updateSlider(const QString&);
    void updateValueOfText(int);
    void toggle_subscribe(bool);
    void generate_mesh();
    void setAngle(const QString&);
    void getDebug();

private:
    QGroupBox* createOpModeBox();
    QGroupBox* createLFBox();

    int minimum_ag, maximum_ag, minimum_rg, maximum_rg;
    int min_slider_lst, max_slider_lst;
    QPushButton*    pause_btn;
    QPushButton*    debug_btn;
    QLineEdit*      max_ag;
    QLineEdit*      min_ag;
    QLineEdit*      v_range;
    QSlider*        min_ag_slider;
    QSlider*        max_ag_slider;
    QSlider*        v_range_slider;
    MyViz*          _viz;

};

} // namespace lidar_base
// END_TUTORIAL
#endif // MYVIZ_H
