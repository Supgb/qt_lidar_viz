#ifndef LIDARINFO_H
#define LIDARINFO_H

#include <QWidget>

#include "../include/lidarctrlpanel.h"

class QGridLayout;
class QListView;
class QScrollBar;

namespace lidar_base
{

class LidarInfo final: public QWidget, public LidarCtrlPanel
{
    Q_OBJECT
public:    
    explicit LidarInfo(_ld_assembler*, _ld_driver_t*,
                       _ld_decoder_t*, _ld_model_t*);
    LidarInfo(const LidarInfo&) = delete;
    LidarInfo& operator=(const LidarInfo&) = delete;
    virtual ~LidarInfo() = default;

    virtual void ui_constructor();

public Q_SLOTS:
    void reloadLogInfo();

private:
    QGridLayout* grid_layout;
    QListView* log_view;
    QScrollBar* scr_bar;
};

}   // namespace lidar_base
#endif // LIDARINFO_H
