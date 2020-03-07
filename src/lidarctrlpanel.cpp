#include "../include/lidarctrlpanel.h"

namespace lidar_base
{

LidarCtrlPanel::LidarCtrlPanel(_ld_assembler* assem,
                               _ld_driver_t* driver,
                               _ld_decoder_t* decoder,
                               _ld_model_t* model)
{
    _assembler = assem;
    _driver = driver;
    _decoder = decoder;
    _model = model;
}

}   // namespace lidar_base被
