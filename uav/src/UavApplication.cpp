//  created:    2025/03/02
//  filename:   UavApplication.cpp
//
//  author:     ateveraz
//
//  version:    $Id: 0.1$
//
//  purpose:    Custom control template
//
//
/*********************************************************************/

#include "UavApplication.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <GroupBox.h>
#include <ComboBox.h>
#include <CheckBox.h>
#include <DoubleSpinBox.h>
#include <Vector3DSpinBox.h>
#include "CustomControl.h"
#include <iostream>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

UavApplication::UavApplication(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false), controlMode_t(ControlMode_t::Default) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
        targetVrpn=new MetaVrpnObject("target",1);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("target");
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("target");
    }
    
    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
      SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    getFrameworkManager()->AddDeviceToLog(targetVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);

    // Groupbox for control selection
    controlModeBox = new GroupBox(GetButtonsLayout()->NewRow(),"Control mode");
    control_selection = new ComboBox(controlModeBox->NewRow(), "Control selection");
    control_selection->AddItem("Default");
    control_selection->AddItem("Custom controller");
    on_controller = new PushButton(controlModeBox->NewRow(), "Activate");
    off_controller = new PushButton(controlModeBox->NewRow(), "Use default");
    positionHold = new PushButton(controlModeBox->NewRow(),"Position hold");

    // Groupbox for task selection
    taskModeBox = new GroupBox(GetButtonsLayout()->LastRowLastCol(), "Task mode");
    task_selection = new ComboBox(taskModeBox->NewRow(), "Task selection");
    task_selection->AddItem("Hovering at zero");
    task_selection->AddItem("Regulation task");
    task_selection->AddItem("Circle tracking");

    desired_position = new Vector3DSpinBox(taskModeBox->NewRow(), "Desired position [m]", -3, 3, 0.1, 3);
    desired_yaw = new DoubleSpinBox(taskModeBox->NewRow(), "Desired yaw [rad]", -M_PI, M_PI, 0.1, 3);

    circle=new TrajectoryGenerator2DCircle(GetButtonsLayout()->NewRow(),"circle");
    uavVrpn->xPlot()->AddCurve(circle->GetMatrix()->Element(0,0),DataPlot::Blue);
    uavVrpn->yPlot()->AddCurve(circle->GetMatrix()->Element(0,1),DataPlot::Blue);
    uavVrpn->VxPlot()->AddCurve(circle->GetMatrix()->Element(1,0),DataPlot::Blue);
    uavVrpn->VyPlot()->AddCurve(circle->GetMatrix()->Element(1,1),DataPlot::Blue);
    uavVrpn->XyPlot()->AddCurve(circle->GetMatrix()->Element(0,1),circle->GetMatrix()->Element(0,0),DataPlot::Blue,"circle");

    uX=new Pid(setupLawTab->At(1,0),"u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY=new Pid(setupLawTab->At(1,1),"u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());

    // Custom control law
    Tab *setup_custom_controller = new Tab(getFrameworkManager()->GetTabWidget(),"Custom controller");
    myCtrl = new CustomControl(setup_custom_controller->At(0,0),"Parameters");

    customReferenceOrientation= new AhrsData(this,"reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);
    AddDeviceToControlLawLog(uX);
    AddDeviceToControlLawLog(uY);
    AddDeviceToControlLawLog(myCtrl);

    customOrientation=new AhrsData(this,"orientation");
}

UavApplication::~UavApplication() {
}

void UavApplication::ComputeCustomTorques(Euler &torques)
{
    // Implement your custom control law here or call a controller class. 
    ComputeDefaultTorques(torques);
    thrust = ComputeDefaultThrust();

    // Selection of the control mode based on the control_selection combobox. 
    switch (control_selection->CurrentIndex())
    {
        case 1:
            controlMode_t = ControlMode_t::Custom;
            computeMyCtrl(torques);
            ComputeCustomThrust();
            break;

        default:
            controlMode_t = ControlMode_t::Default;
            Thread::Warn("UavApplication: default control law started. Check custom torque definition. \n");
            EnterFailSafeMode();
            break;
    }
}

void UavApplication::computeMyCtrl(Euler &torques)
{
    // Get position, velocity and quaternion from the VRPN object in its coordinate system
    Vector3Df uav_pos, uav_vel; 
    Quaternion vrpn_quaternion;
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    uavVrpn->GetQuaternion(vrpn_quaternion);

    // Get current orientation and angular speed from the AHRS object (IMU)
    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();

    // Use yaw from VRPN and roll, pitch from IMU
    Euler ahrsEuler = currentQuaternion.ToEuler();
    ahrsEuler.yaw = vrpn_quaternion.ToEuler().yaw;
    Quaternion mixQuaternion = ahrsEuler.ToQuaternion();

    // Compute the position and velocity errors in the UAV frame
    Vector2Df pos_error2D, vel_error2D;
    // Example of desired altitude [m] => (ALWAYS A POSITIVE VALUE) 
    // Because the AltitudeValues function returns a positive value also. However, the UAV's altitude is negative in the VRPN coordinate system.
    float altittude_desired = desired_position->Value().z; 
    if (altittude_desired <= 0)
    {
        Thread::Warn("UavApplication: desired altitude must be positive, see AltitudeValues function.\n");
        EnterFailSafeMode();
        return;
    }
    float yaw_ref;
    float z, dz;
    AltitudeValues(z, dz);
    PositionValues(pos_error2D, vel_error2D, yaw_ref);
    // Notice that the error definition is current - desired for x,y and z. 
    Vector3Df pos_error = Vector3Df(pos_error2D.x, pos_error2D.y, z-altittude_desired);
    Vector3Df vel_error = Vector3Df(vel_error2D.x, vel_error2D.y, dz);

    // Set the values of the custom controller and update it
    myCtrl->SetValues(pos_error, vel_error, mixQuaternion, currentAngularSpeed, yaw_ref);
    myCtrl->Update(GetTime());

    // Apply the computed torques and thrust
    torques.roll = myCtrl->Output(0);
    torques.pitch = myCtrl->Output(1);
    torques.yaw = myCtrl->Output(2);
    thrust = myCtrl->Output(3); 
    // If you prefer, you can also use the ComputeDefaultThrust() function. E.g.:
    // thrust = ComputeDefaultThrust();
    // The desired take-off altitude will be used as a reference. 
}

float UavApplication::ComputeCustomThrust(void)
{
    // Implement your custom thrust computation here or asign its value from another function, because it is a global variable.
    if (thrust == 0)
    {
        // For safety reasons, the default thrust is computed if the custom thrust is not defined.
        thrust = ComputeDefaultThrust();
        std::cout << "Custom thrust not defined, using default thrust: " << thrust << std::endl;
    }
    return thrust;
}

void UavApplication::StartController(void)
{
    if (control_selection->CurrentIndex() == 0)
    {
        StartDefaultTorques();
        Start_task();
        Thread::Info("UavApplication: default control law started\n");
    }
    else
    {
        if(SetTorqueMode(TorqueMode_t::Custom) && SetThrustMode(ThrustMode_t::Custom) && control_selection->CurrentIndex() != 0)
        {
            controlMode_t = ControlMode_t::Custom;
            myCtrl->Reset();
            Start_task();
            Thread::Info("UavApplication: custom control law started\n");
        }
        else
        {
            StopCustomController();
            Thread::Err("UavApplication: could not start custom control law\n");
        }
    }
    
}

void UavApplication::StopCustomController(void)
{
    StartDefaultTorques();
    EnterFailSafeMode();
    controlMode_t = ControlMode_t::Default;
    Thread::Info("UavApplication: failing safe mode activated, custom control law stopped\n");
}

void UavApplication::StartDefaultTorques(void)
{
    if (controlMode_t == ControlMode_t::Default)
    {
        Thread::Warn("UavApplication: already in default control law\n");
        return;
    }

    if(SetTorqueMode(TorqueMode_t::Default) && SetThrustMode(ThrustMode_t::Default) )
    {
        controlMode_t = ControlMode_t::Default;
        Thread::Info("UavApplication: default control law started\n");
    }
    else
    {
        Thread::Err("UavApplication: could not start default control law\n");
    }
}

const AhrsData *UavApplication::GetOrientation(void) const {
    //get yaw from vrpn
    Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);

    //get roll, pitch and w from imu
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    // yaw from vrpn and roll, pitch from imu
    Euler ahrsEuler=ahrsQuaternion.ToEuler();
    ahrsEuler.yaw=vrpnQuaternion.ToEuler().yaw;
    Quaternion mixQuaternion=ahrsEuler.ToQuaternion();

    customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);

    return customOrientation;
}

void UavApplication::AltitudeValues(float &z,float &dz) const{
    Vector3Df uav_pos,uav_vel;

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    //z and dz must be in uav's frame
    z=-uav_pos.z;
    dz=-uav_vel.z;
}

AhrsData *UavApplication::GetReferenceOrientation(void) {
    Vector2Df pos_err, vel_err; // in Uav coordinate system
    float yaw_ref;
    Euler refAngles;

    PositionValues(pos_err, vel_err, yaw_ref);

    refAngles.yaw=yaw_ref;

    uX->SetValues(pos_err.x, vel_err.x);
    uX->Update(GetTime());
    refAngles.pitch=uX->Output();

    uY->SetValues(pos_err.y, vel_err.y);
    uY->Update(GetTime());
    refAngles.roll=-uY->Output();

    customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(),Vector3Df(0,0,0));

    return customReferenceOrientation;
}

void UavApplication::PositionValues(Vector2Df &pos_error,Vector2Df &vel_error,float &yaw_ref) {
    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Vector2Df uav_2Dpos,uav_2Dvel; // in VRPN coordinate system

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);

    uav_pos.To2Dxy(uav_2Dpos);
    uav_vel.To2Dxy(uav_2Dvel);

    if (behaviourMode==BehaviourMode_t::PositionHold) {
        pos_error=uav_2Dpos-posHold;
        vel_error=uav_2Dvel;
        yaw_ref=yawHold;
    } 
    else if (behaviourMode==BehaviourMode_t::Hover) {
        pos_error=uav_2Dpos;
        vel_error=uav_2Dvel;
        yaw_ref=0;
    } 
    else if (behaviourMode==BehaviourMode_t::Regulation) {
        Vector2Df desired_position_xy(desired_position->Value().x, desired_position->Value().y);
        pos_error=uav_2Dpos - desired_position_xy;
        vel_error=uav_2Dvel;
        yaw_ref=(float)desired_yaw->Value();
    }
    else { //Circle
        Vector3Df target_pos;
        Vector2Df circle_pos,circle_vel;
        Vector2Df target_2Dpos;

        targetVrpn->GetPosition(target_pos);
        target_pos.To2Dxy(target_2Dpos);
        circle->SetCenter(target_2Dpos);

        //circle reference
        circle->Update(GetTime());
        circle->GetPosition(circle_pos);
        circle->GetSpeed(circle_vel);

        //error in optitrack frame
        pos_error=uav_2Dpos-circle_pos;
        vel_error=uav_2Dvel-circle_vel;
        yaw_ref=atan2(target_pos.y-uav_pos.y,target_pos.x-uav_pos.x);
    }

    //error in uav frame
    Quaternion currentQuaternion=GetCurrentQuaternion();
    Euler currentAngles;//in vrpn frame
    currentQuaternion.ToEuler(currentAngles);
    pos_error.Rotate(-currentAngles.yaw);
    vel_error.Rotate(-currentAngles.yaw);
}

void UavApplication::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    case Event_t::EnteringControlLoop:
        if ((behaviourMode==BehaviourMode_t::Circle) && (!circle->IsRunning())) {
            VrpnPositionHold();
        }
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}

void UavApplication::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Circle) || (behaviourMode==BehaviourMode_t::PositionHold))) {
        if (!targetVrpn->IsTracked(500)) {
            Thread::Err("VRPN, target lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Hover) || (behaviourMode==BehaviourMode_t::Regulation))) {
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void UavApplication::ExtraCheckPushButton(void) 
{
    if(on_controller->Clicked()) {
        StartController();
    } 
    if(off_controller->Clicked()) {
        StopCustomController();
    }
    if(positionHold->Clicked()) {
        VrpnPositionHold();
    }
}

void UavApplication::ExtraCheckJoystick(void) {
    /*     Do not use cross, start nor select buttons!!
    0: "start"       1: "select"      2: "square"      3: "triangle"
    4: "circle"      5: "cross";      6: "left 1"      7: "left 2"
    8: "left 3"      9: "right 1"     10: "right 2"    11: "right 3"
    12: "up"         13: "down"       14: "left"       15: "right"
    */

    //R1 and Circle
    if(GetTargetController()->ButtonClicked(4) && GetTargetController()->IsButtonPressed(9)) {
        StartController();
    }
    
    //R1 and Square
    if(GetTargetController()->ButtonClicked(2) && GetTargetController()->IsButtonPressed(9)) {
        VrpnPositionHold();
    }
}

void UavApplication::Start_task(void) {
    if( behaviourMode==BehaviourMode_t::Circle) {
        Thread::Warn("UavApplication: already in circle mode\n");
        return;
    }
    if (SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Info("UavApplication: starting custom task ...\n");
    } else {
        Thread::Warn("UavApplication: could not start custom task\n");
        return;
    }

    // Defining desired task. 
    if (task_selection->CurrentIndex() == 0) {
        behaviourMode=BehaviourMode_t::Hover;
        Thread::Info("UavApplication: hovering at zero\n");
    }
    else if (task_selection->CurrentIndex() == 1) {
        behaviourMode=BehaviourMode_t::Regulation;
        Thread::Info("UavApplication: regulation task\n");
    }
    else if (task_selection->CurrentIndex() == 2) {
        Vector3Df uav_pos,target_pos;
        Vector2Df uav_2Dpos,target_2Dpos;

        targetVrpn->GetPosition(target_pos);
        target_pos.To2Dxy(target_2Dpos);
        circle->SetCenter(target_2Dpos);

        uavVrpn->GetPosition(uav_pos);
        uav_pos.To2Dxy(uav_2Dpos);
        circle->StartTraj(uav_2Dpos);

        uX->Reset();
        uY->Reset();
        behaviourMode=BehaviourMode_t::Circle;
        Thread::Info("UavApplication: circle tracking\n");
    }
    else {
        EnterFailSafeMode();
        Thread::Err("UavApplication: unknown task\n");
        return;
    }
}

void UavApplication::StopCircle(void) {
    if( behaviourMode!=BehaviourMode_t::Circle) {
        Thread::Warn("UavApplication: not in circle mode\n");
        return;
    }
    circle->FinishTraj();
    //GetJoystick()->Rumble(0x70);
    Thread::Info("UavApplication: finishing circle\n");
}

void UavApplication::VrpnPositionHold(void) {
    if( behaviourMode==BehaviourMode_t::PositionHold) {
        Thread::Warn("UavApplication: already in vrpn position hold mode\n");
        return;
    }
	Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);
	yawHold=vrpnQuaternion.ToEuler().yaw;

    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::PositionHold;
    SetOrientationMode(OrientationMode_t::Custom);
    Thread::Info("UavApplication: holding position\n");
}
