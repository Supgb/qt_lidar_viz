#ifndef LIDARCTRLPANEL_H
#define LIDARCTRLPANEL_H

#include "../include/lidar_assembler.h"
#include "../include/lidardecoder.h"
#include "../include/lidardriver.h"
#include "../include/lidarmdl.h"

namespace lidar_base {

class LidarCtrlPanel
{
public:
    typedef lidar_base::LidarAssembler::ExecMode _Mode;
    typedef lidar_base::LidarAssembler _ld_assembler;
    typedef lidar_driver::LidarDriver _ld_driver_t;
    typedef lidar_driver::LidarDecoder _ld_decoder_t;
    typedef lidar_base::LidarMDL _ld_model_t;

    explicit LidarCtrlPanel(_ld_assembler *assem,
                            _ld_driver_t* driver,
                            _ld_decoder_t* decoder,
                            _ld_model_t* model);
    LidarCtrlPanel(const LidarCtrlPanel&) = delete;
    LidarCtrlPanel& operator=(const LidarCtrlPanel&) = delete;
    virtual ~LidarCtrlPanel() = default;

    virtual void ui_constructor() = 0;

public:
    _ld_assembler* _assembler;
    _ld_driver_t* _driver;
    _ld_decoder_t* _decoder;
    _ld_model_t* _model;
};

}   // namespace lidar_base

#endif // LIDARCTRLPANEL_H
