#ifndef LIDARCTRLPANEL_H
#define LIDARCTRLPANEL_H

#include "../include/lidar_assembler.h"

namespace lidar_ctrl {

class LidarCtrlPanel
{
public:
    typedef lidar_assembler::LidarAssembler::ExecMode _Mode;
    typedef lidar_assembler::LidarAssembler _ld_assembler;

    explicit LidarCtrlPanel(_ld_assembler *assem);
    LidarCtrlPanel(const LidarCtrlPanel&) = delete;
    LidarCtrlPanel& operator=(const LidarCtrlPanel&) = delete;
    virtual ~LidarCtrlPanel(){}

    virtual void ui_constructor() = 0;

public:
    _ld_assembler* _assembler;
};

}   // namespace lidar_ctrl

#endif // LIDARCTRLPANEL_H
