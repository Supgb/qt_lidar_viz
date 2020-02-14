#ifndef LIDARINFO_H
#define LIDARINFO_H

#include <QWidget>
#include <QGridLayout>
#include <QListView>
#include <QScrollBar>

#include "../include/lidarctrlpanel.h"
#include "../include/lidartune.h"

namespace lidar_ctrl
{

class LidarInfo : public QWidget, public LidarCtrlPanel
{
    Q_OBJECT
public:    
    explicit LidarInfo(_ld_assembler *assem);
    LidarInfo(const LidarInfo&) = delete;
    LidarInfo& operator=(const LidarInfo&) = delete;
    virtual ~LidarInfo(){}

    virtual void ui_constructor();

public Q_SLOTS:
    void reloadLogInfo();

private:
    QGridLayout* grid_layout;
    QListView* log_view;
    QScrollBar* scr_bar;
};

}
#endif // LIDARINFO_H
