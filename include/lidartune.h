#ifndef LIDARTUNE_H
#define LIDARTUNE_H

#include <QWidget>
#include "../include/lidarctrlpanel.h"

class QGridLayout;
class QComboBox;
class QPushButton;
class QLineEdit;
class QSlider;
class QGroupBox;

class MyViz;

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
    void changeTo3DModel();

private:
    QGroupBox* createOpModeBox();
    QGroupBox* createLFBox();

    int minimum_ag, maximum_ag, minimum_rg, maximum_rg;
    int min_slider_lst, max_slider_lst;
    QPushButton*    pause_btn;
    QLineEdit*      max_ag;
    QLineEdit*      min_ag;
    QLineEdit*      v_range;
    QSlider*        min_ag_slider;
    QSlider*        max_ag_slider;
    QSlider*        v_range_slider;
    MyViz*          _viz;

};

} // namespace lidar_base

#endif // LIDARTUNE_H
