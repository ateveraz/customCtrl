//  created:    2011/05/01
//  filename:   customCtrl.h
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo cercle avec optitrack
//
//
/*********************************************************************/

#ifndef CIRCLEFOLLOWER_H
#define CIRCLEFOLLOWER_H

#include <UavStateMachine.h>

namespace flair {
    namespace gui {
        class PushButton;
    }
    namespace filter {
        class TrajectoryGenerator2DCircle;
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
            Circle
        };

        BehaviourMode_t behaviourMode;
        bool vrpnLost;

        void VrpnPositionHold(void);//flight mode
        void StartCircle(void);
        void StopCircle(void);
        void ExtraSecurityCheck(void) override;
        void ExtraCheckPushButton(void) override;
        void ExtraCheckJoystick(void) override;
        const flair::core::AhrsData *GetOrientation(void) const override;
        void AltitudeValues(float &z,float &dz) const override;
        void PositionValues(flair::core::Vector2Df &pos_error,flair::core::Vector2Df &vel_error,float &yaw_ref);
        flair::core::AhrsData *GetReferenceOrientation(void) override;
        void SignalEvent(Event_t event) override;

        flair::filter::Pid *uX, *uY;

        flair::core::Vector2Df posHold;
        float yawHold;

        flair::gui::PushButton *startCircle,*stopCircle,*positionHold;
        flair::meta::MetaVrpnObject *targetVrpn,*uavVrpn;
        flair::filter::TrajectoryGenerator2DCircle *circle;
        flair::core::AhrsData *customReferenceOrientation,*customOrientation;
};

#endif // CIRCLEFOLLOWER_H
