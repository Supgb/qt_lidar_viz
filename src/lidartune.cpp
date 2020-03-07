#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSpacerItem>
#include <QLineEdit>
#include <QMessageBox>
#include <QGridLayout>
#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>
#include <QSlider>
#include <QGroupBox>

#include "../include/myviz.h"
#include "../include/lidardebug.h"

#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/default_plugin/marker_display.h"
#include "OGRE/OgreMeshManager.h"

#include "boost/thread.hpp"

namespace lidar_base
{

LidarTune::LidarTune(_ld_assembler* assem, _ld_driver_t* driver,
                     _ld_decoder_t* decoder, _ld_model_t* model, MyViz* viz):
    LidarCtrlPanel(assem, driver, decoder, model),
    minimum_ag(0), maximum_ag(35999),
    minimum_rg(0), maximum_rg(10000),
    pause_btn(new QPushButton()),
    debug_btn(new QPushButton()),
    max_ag(new QLineEdit()),
    min_ag(new QLineEdit()),
    v_range(new QLineEdit()),
    min_ag_slider(new QSlider(Qt::Horizontal)),
    max_ag_slider(new QSlider(Qt::Horizontal)),
    v_range_slider(new QSlider(Qt::Horizontal)),
    min_slider_lst(0), max_slider_lst(0),
    _viz(viz)
{
    ui_constructor();
}

void LidarTune::ui_constructor()
{
    QGridLayout* grid_layout = new QGridLayout();
    QGroupBox* op_mode_box = createOpModeBox();
    QGroupBox* lidar_filter_box = createLFBox();

    QSpacerItem* h_spacer = new QSpacerItem(350, 0, QSizePolicy::Expanding, QSizePolicy::Minimum);
    QSpacerItem* v_spacer = new QSpacerItem(21, 350, QSizePolicy::Minimum, QSizePolicy::Expanding);

    grid_layout->addWidget(op_mode_box, 0, 0);
    grid_layout->addWidget(lidar_filter_box, 1, 0);

    setLayout(grid_layout);

    QObject::connect(op_mode_box->findChild<QComboBox*>("exec_mode_box"), SIGNAL(currentTextChanged(const QString&)),
                    _assembler, SLOT(setExecMode(const QString&)),
                    Qt::QueuedConnection);
}

QGroupBox* LidarTune::createOpModeBox()
{
    QGroupBox* op_mode_box = new QGroupBox("Operating groups");
    op_mode_box->setMaximumHeight(120);

    QLabel* execMode_lab = new QLabel("<b>Data flow mode:</b>");
    QLabel* create_mesh_lab = new QLabel("<b>Create mesh:</b>");

    QComboBox* execMode_box = new QComboBox();
    execMode_box->setObjectName("exec_mode_box");
    execMode_box->addItem("LaserScan");
    execMode_box->addItem("Storage");
    execMode_box->addItem("Assemble");
    //execMode_box->setMaximumWidth(100);

    QPushButton* generate_btn = new QPushButton();
    generate_btn->setText("Generate");
    generate_btn->setToolTip("generate the last mesh");
    //generate_btn->setMaximumWidth(70);

    QGridLayout* grid_layout = new QGridLayout;
    grid_layout->addWidget(execMode_lab, 0, 0);
    grid_layout->addWidget(execMode_box, 0, 1);
    grid_layout->addWidget(create_mesh_lab, 1, 0);
    grid_layout->addWidget(generate_btn, 1, 1);
    op_mode_box->setLayout(grid_layout);

    connect ( generate_btn, SIGNAL( released() ), this, SLOT(generate_mesh()));

    return op_mode_box;
}


QGroupBox* LidarTune::createLFBox()
{
    QGroupBox* lidar_filter_box = new QGroupBox("Lidar Filters");

    QLabel* max_ag_lab = new QLabel("<b>Max Angle:</b>");
    QLabel* min_ag_lab = new QLabel("<b>Min Angle:</b>");
    QLabel* v_range_lab = new QLabel("<b>Vertical Range:</b>");
    QLabel* debug_lab = new QLabel("<b>Debug</b");

    max_ag->setMaximumHeight(20);
    max_ag->setToolTip("Up to 360");
    min_ag->setMaximumHeight(20);
    min_ag->setToolTip("at least 0");
    v_range->setMaximumHeight(20);
    v_range->setToolTip("The maximum in-duty distance");

    min_ag_slider->setMinimum(minimum_ag);
    min_ag_slider->setMaximum(maximum_ag);
    max_ag_slider->setMinimum(minimum_ag);
    max_ag_slider->setMaximum(maximum_ag);
    v_range_slider->setMinimum(minimum_rg);
    v_range_slider->setMaximum(maximum_rg);

    pause_btn->setCheckable(true);
    pause_btn->setText("Pause");
    pause_btn->setToolTip("Pause scan...");

    debug_btn->setText("Debug");
    debug_btn->setToolTip("Generate debug info...");

    QVBoxLayout* v_layout = new QVBoxLayout;

    QHBoxLayout* h_layout_min = new QHBoxLayout;
    QHBoxLayout* h_layout_max = new QHBoxLayout;
    QHBoxLayout* h_layout_vrange = new QHBoxLayout;

    h_layout_min->addWidget(min_ag_lab);
    h_layout_min->addWidget(min_ag);
    h_layout_min->addStretch(1);
    v_layout->addLayout(h_layout_min);
    v_layout->addWidget(min_ag_slider);

    h_layout_max->addWidget(max_ag_lab);
    h_layout_max->addWidget(max_ag);
    h_layout_max->addStretch(1);
    v_layout->addLayout(h_layout_max);
    v_layout->addWidget(max_ag_slider);

    h_layout_vrange->addWidget(v_range_lab);
    h_layout_vrange->addWidget(v_range);
    h_layout_vrange->addStretch(1);
    v_layout->addLayout(h_layout_vrange);
    v_layout->addWidget(v_range_slider);

    v_layout->addWidget(pause_btn);
    v_layout->addWidget(debug_btn);

    v_layout->addStretch(1);
    lidar_filter_box->setLayout(v_layout);

    connect ( max_ag_slider, SIGNAL( valueChanged(int) ), this, SLOT( updateValueOfText(int) ) );
    connect ( max_ag, SIGNAL( textChanged(const QString&)), this, SLOT( setAngle(const QString&)));
    connect ( max_ag, SIGNAL( textChanged(const QString&)), this, SLOT( updateSlider(const QString&)));
    connect ( min_ag_slider, SIGNAL( valueChanged(int) ), this, SLOT( updateValueOfText(int) ) );
    connect ( min_ag, SIGNAL( textChanged(const QString&)), this, SLOT( setAngle(const QString&)));
    connect ( min_ag, SIGNAL( textChanged(const QString&)), this, SLOT( updateSlider(const QString&)));
    connect ( pause_btn, SIGNAL( toggled( bool )), this, SLOT(toggle_subscribe(bool)));
    connect ( debug_btn, SIGNAL( released() ), this, SLOT(getDebug()));
    //connect ( debug_btn, SIGNAL( released() ), _model, SLOT(outputDebug()), Qt::QueuedConnection);

    min_ag_slider->setValue(0);
    max_ag_slider->setValue(35999);

    return lidar_filter_box;
}

inline bool LidarTune::checkValid()
{
    return (max_ag_slider->value() > min_ag_slider->value())||(max_ag_slider->value() == min_ag_slider->value()) ? true : false;
}

inline bool LidarTune::checkValid(double value)
{
    return ((value < 359.99) || (value == 359.99)) && ((value > 0) || (value == 0)) ? true : false;
}

void LidarTune::updateSlider(const QString &angle_str)
{
    QLineEdit* obj = (QLineEdit*)QObject::sender();
    double value = angle_str.toDouble();
    if(!checkValid(value))
    {
        QMessageBox* failed = new QMessageBox(
                            "Invalid Operation",
                            "The minimum value of angle can not be greater than the maximum value.",
                            QMessageBox::Information,
                            QMessageBox::Ok | QMessageBox::Escape, 0, 0);
                failed->show();
        return;
    }
    if (obj == max_ag)
    {
        max_ag_slider->blockSignals(true);
        max_ag_slider->setValue(static_cast<int>(value*100));
        max_ag_slider->blockSignals(false);
    }
    else if (obj == min_ag)
    {
        min_ag_slider->blockSignals(true);
        min_ag_slider->setValue(static_cast<int>(value*100));
        min_ag_slider->blockSignals(false);
    }
}

void LidarTune::updateValueOfText(int value)
{
    QSlider* obj = (QSlider*)QObject::sender();
    //QString str = QString::number(obj->value()/100.0);
    QString str = QString::number(value/100.0);
    if(!checkValid())
    {
        if(obj == max_ag_slider)
            obj->setValue(max_slider_lst);
        else if (obj == min_ag_slider)
            obj->setValue(min_slider_lst);
        QMessageBox* failed = new QMessageBox(
                    "Invalid Operation",
                    "The minimum value of angle can not be greater than the maximum value.",
                    QMessageBox::Information,
                    QMessageBox::Ok | QMessageBox::Escape, 0, 0);
        failed->show();
        return;
    }
    if (obj == max_ag_slider)
    {
        max_ag->setText(str);
        //max_slider_lst = obj->value();
        max_slider_lst = value;
    }
    else if (obj == min_ag_slider)
    {
        min_ag->setText(str);
        //min_slider_lst = obj->value();
        min_slider_lst = value;
    }
}

void LidarTune::setAngle(const QString& str_angle)
{
    QObject* obj = QObject::sender();
    if (obj == max_ag)
    {
        _decoder->setMaxAngle(str_angle.toDouble());
    }
    else if (obj == min_ag)
    {
        _decoder->setMinAngle(str_angle.toDouble());
    }

}

void LidarTune::toggle_subscribe(bool checked)
{
    if ( checked )
    {
        pause_btn->setText("Resume");
        //_model->_unsubscribe();
        _assembler->_unsubscribe();
    } else
    {
        pause_btn->setText("Pause");
        //_model->_resubscribe();
        _assembler->_resubscribe();
    }
}

void LidarTune::generate_mesh()
{
    _model->__reconstruct();
    _viz->getScan()->setEnabled(false);
    _viz->getMarker()->setEnabled(true);
    _viz->resetMesh();
}

void LidarTune::getDebug()
{
    _viz->setBoostDebugger(_viz->getDebugger()->spawn());
}

}   // namespace lidar_base
