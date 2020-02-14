#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSpacerItem>
#include <QLineEdit>

#include "../include/lidartune.h"

using namespace lidar_ctrl;

LidarTune::LidarTune(_ld_assembler *assem):
    LidarCtrlPanel(assem)
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

    return op_mode_box;
}


QGroupBox* LidarTune::createLFBox()
{
    QGroupBox* lidar_filter_box = new QGroupBox("Lidar Filters");

    QLabel* max_ag_lab = new QLabel("<b>Max Angle:</b>");
    QLabel* min_ag_lab = new QLabel("<b>Min Angle:</b>");
    QLabel* v_range_lab = new QLabel("<b>Vertical Range:</b>");

    QLineEdit* max_ag = new QLineEdit();
    max_ag->setMaximumHeight(20);
    max_ag->setToolTip("Up to 360");
    QLineEdit* min_ag = new QLineEdit();
    min_ag->setMaximumHeight(20);
    min_ag->setToolTip("at least 0");
    QLineEdit* v_range = new QLineEdit();
    v_range->setMaximumHeight(20);
    v_range->setToolTip("The maximum in-duty distance");

    QSlider* min_ag_slider = new QSlider(Qt::Horizontal);
    min_ag_slider->setMinimum(0);
    min_ag_slider->setMaximum(360);
    QSlider* max_ag_slider = new QSlider(Qt::Horizontal);
    max_ag_slider->setMinimum(0);
    max_ag_slider->setMaximum(360);
    QSlider* v_range_slider = new QSlider(Qt::Horizontal);
    v_range_slider->setMinimum(0);
    v_range_slider->setMaximum(100);

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

    v_layout->addStretch(1);
    lidar_filter_box->setLayout(v_layout);

    return lidar_filter_box;
}
