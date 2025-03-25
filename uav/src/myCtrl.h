#ifndef MYCTRL_H
#define MYCTRL_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>
#include <Quaternion.h>
#include <Pid.h>

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
        class CheckBox;
        class Label;
        class Vector3DSpinBox;
    }
    namespace filter {
        class Pid;
    }
}

namespace flair {
    namespace filter {
        class MyController : public ControlLaw
        {
            public :
                MyController(const flair::gui::LayoutPosition *position, const std::string &name);
                ~MyController();
                void UpdateFrom(const flair::core::io_data *data);
                void Reset(void);
                void SetValues(flair::core::Vector3Df pos_error, flair::core::Vector3Df vel_error, flair::core::Quaternion currentQuaternion, flair::core::Vector3Df omega, flair::core::Quaternion qz);
                void Saturate(flair::core::Vector3Df &vec, flair::core::Vector3Df sat);

            private : 
                float delta_t, initial_time;
                float g = 9.81;
                bool first_update;

                flair::core::Matrix *state;
                flair::gui::Vector3DSpinBox *Kp_pos, *Kd_pos, *Ki_pos, *Kp_att, *Kd_att, *Ki_att;
                flair::gui::Vector3DSpinBox *sat_pos, *sat_att;
                flair::gui::DoubleSpinBox *deltaT_custom, *mass;

                flair::filter::Pid *u_x, *u_y, *u_z;
                flair::filter::Pid *u_roll, *u_pitch, *u_yaw;


        };
    }
}

#endif // MYCTRL_H