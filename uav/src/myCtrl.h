#ifndef MYCTRL_H
#define MYCTRL_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>
#include <Quaternion.h>

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
        // If you prefer to use a custom controller class, you can define it here.
        // ...
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
                void SetValues(const flair::core::Vector3Df &pos_error, const flair::core::Vector3Df &vel_error, const flair::core::Quaternion &currentQuaternion, const flair::core::Vector3Df &omega, float yaw_ref);
                void applyMotorConstant(flair::core::Vector3Df &signal);
                void applyMotorConstant(float &signal);

            private : 
                float delta_t, initial_time;
                float g = 9.81;
                bool first_update;

                flair::core::Matrix *state;
                flair::gui::Vector3DSpinBox *Kp_pos, *Kd_pos, *Kp_att, *Kd_att ;
                flair::gui::DoubleSpinBox *deltaT_custom, *mass, *k_motor, *sat_pos, *sat_att, *sat_thrust;

                void plotCartesianErrors(const flair::gui::LayoutPosition *position);
        };
    }
}

#endif // MYCTRL_H