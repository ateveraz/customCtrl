//  created:    2025/03/02
//  filename:   customCtrl.h
//
//  author:     ateveraz
//
//  version:    $Id: 0.1$
//
//  purpose:    Custom control template
//
//
/*********************************************************************/

#ifndef CUSTOMCTRL_H
#define CUSTOMCTRL_H

#include <UavStateMachine.h>
#include "myCtrl.h"

namespace flair {
    namespace gui {
        class PushButton;
        class GroupBox;
        class ComboBox;
        class CheckBox;
        class Vector3DSpinBox;
        class DoubleSpinBox;
    }
    namespace filter {
        class TrajectoryGenerator2DCircle;
        class MyController;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
}

class customCtrl : public flair::meta::UavStateMachine {
    public:
        customCtrl(flair::sensor::TargetController *controller);
        ~customCtrl();

    private:

	enum class BehaviourMode_t {
            Default,
            PositionHold,
            Circle, 
            Regulation, 
            Hover
        };

    enum class ControlMode_t {
            Default, 
            Custom
    };

        BehaviourMode_t behaviourMode;
        ControlMode_t controlMode_t;
        bool vrpnLost;

        void VrpnPositionHold(void);//flight mode
        void Start_task(void);
        void StopCircle(void);
        void ExtraSecurityCheck(void) override;
        void ExtraCheckPushButton(void) override;
        void ExtraCheckJoystick(void) override;
        const flair::core::AhrsData *GetOrientation(void) const override;
        void AltitudeValues(float &z,float &dz) const override;
        void PositionValues(flair::core::Vector2Df &pos_error,flair::core::Vector2Df &vel_error,float &yaw_ref);
        flair::core::AhrsData *GetReferenceOrientation(void) override;
        void SignalEvent(Event_t event) override;
        void ComputeCustomTorques(flair::core::Euler &torques);
        float ComputeCustomThrust(void);
        void StartCustomTorques(void);
        void StopCustomTorques(void);
        void StartDefaultTorques(void);
        void computeMyCtrl(flair::core::Euler &torques);

        flair::filter::Pid *uX, *uY;
        flair::filter::MyController *myCtrl;

        flair::core::Vector2Df posHold;
        float yawHold;
        float thrust;

        flair::gui::PushButton *startCircle,*stopCircle,*positionHold;
        flair::meta::MetaVrpnObject *targetVrpn,*uavVrpn;
        flair::filter::TrajectoryGenerator2DCircle *circle;
        flair::core::AhrsData *customReferenceOrientation,*customOrientation;

        // Control mode GUI
        flair::gui::GroupBox *controlModeBox;
        flair::gui::ComboBox *control_selection;
        flair::gui::PushButton *on_customController, *off_customController;

        // Custom control law
        flair::gui::DoubleSpinBox *deltaT_custom;

        // Custom task
        flair::gui::ComboBox *task_selection;
        flair::gui::Vector3DSpinBox *desired_position;
        flair::gui::DoubleSpinBox *desired_yaw;
};

#endif // CUSTOMCTRL_H
