#ifndef LIDARTUNE_H
#define LIDARTUNE_H

#include <QWidget>
#include <QGridLayout>
#include <QComboBox>
#include <QPushButton>
#include <QGroupBox>

#include "../include/lidarctrlpanel.h"

namespace lidar_ctrl
{

class LidarTune : public QWidget, public LidarCtrlPanel
{
    Q_OBJECT
public:
    explicit LidarTune(_ld_assembler *assem);
    LidarTune(const LidarTune&) = delete;
    LidarTune& operator =(const LidarTune&) = delete;
    virtual ~LidarTune(){}

    virtual void ui_constructor();

private:
    QGroupBox* createOpModeBox();
    QGroupBox* createLFBox();
};

} // namespace lidar_ctrl

#endif // LIDARTUNE_H
