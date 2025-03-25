#include "myCtrl.h"
#include <Matrix.h>
#include <Vector3D.h>
#include <TabWidget.h>
#include <CheckBox.h>
#include <Quaternion.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <cmath>
#include <Euler.h>
#include <iostream>
#include <Label.h>
#include <Vector3DSpinBox.h>
#include <Pid.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

MyController::MyController(const LayoutPosition *position, const string &name) : ControlLaw(position->getLayout(),name,4)
{
    first_update = true;

    // Input matrix
    input = new Matrix(this, 4, 5, floatType, name);

    // Matrix descriptor for logging. 
    MatrixDescriptor *log_labels = new MatrixDescriptor(3, 1);
    log_labels->SetElementName(0, 0, "roll_error");
    log_labels->SetElementName(1, 0, "pitch_error");
    log_labels->SetElementName(2, 0, "yaw_error");
    state = new Matrix(this, log_labels, floatType, name);
    delete log_labels;

    // GUI for custom PD
    GroupBox *gui_customPD = new GroupBox(position, name);
    deltaT_custom = new DoubleSpinBox(gui_customPD->NewRow(), "Custom dt [s]", 0, 1, 0.001, 4);
    mass = new DoubleSpinBox(gui_customPD->NewRow(), "Mass [kg]", 0, 10, 0.01, 3);

    // Custom cartesian position controller
    u_x = new Pid(gui_customPD->NewRow(), "u_x");
    u_y = new Pid(gui_customPD->LastRowLastCol(), "u_y");
    u_z = new Pid(gui_customPD->LastRowLastCol(), "u_z");

    // Custom attitude controller
    u_roll = new Pid(gui_customPD->NewRow(), "u_roll");
    u_pitch = new Pid(gui_customPD->LastRowLastCol(), "u_pitch");
    u_yaw = new Pid(gui_customPD->LastRowLastCol(), "u_yaw");

    AddDataToLog(state);
}

MyController::~MyController()
{
    delete state;
}

void MyController::UpdateFrom(const io_data *data)
{
    float current_time = double(GetTime())/1000000000-initial_time;
    float Tr = 0.0, tau_roll = 0.0, tau_pitch = 0.0, tau_yaw = 0.0;

    if(deltaT_custom->Value() == 0)
    {
        delta_t = (float)(data->DataDeltaTime())/1000000000;
    }
    else
    {
        delta_t = deltaT_custom->Value();
    }
   
    if(first_update)
    {
        initial_time = double(GetTime())/1000000000;
        first_update = false;
    }

    // Obtain state
    input->GetMutex();
    Vector3Df pos_error(input->Value(0, 0), input->Value(1, 0), input->Value(2, 0));
    Vector3Df vel_error(input->Value(0, 1), input->Value(1, 1), input->Value(2, 1));
    Quaternion q(input->Value(0, 2), input->Value(1, 2), input->Value(2, 2), input->Value(3, 2));
    Vector3Df omega(input->Value(0, 3), input->Value(1, 3), input->Value(2, 3));
    Quaternion qz(input->Value(0, 4), input->Value(1, 4), input->Value(2, 4), input->Value(3, 4));
    input->ReleaseMutex();

    u_x->SetValues(pos_error.x, vel_error.x);
    u_x->Update(current_time);
    u_y->SetValues(pos_error.y, vel_error.y);
    u_y->Update(current_time);
    u_z->SetValues(pos_error.z, vel_error.z);
    u_z->Update(current_time);

    Euler rpy = q.ToEuler();
    u_roll->SetValues(rpy.roll + u_y->Output(), omega.x);
    u_roll->Update(current_time);
    u_pitch->SetValues(rpy.pitch - u_x->Output(), omega.y);
    u_pitch->Update(current_time);
    u_yaw->SetValues(rpy.YawDistanceFrom(0), omega.z);
    u_yaw->Update(current_time);
    
    float force_thrust = - u_z->Output() - mass->Value()*g;
    Vector3Df att_control(u_roll->Output(), u_pitch->Output(), u_yaw->Output());

    // std::cout << "error xyz: " << pos_error.x << " " << pos_error.y << " " << pos_error.z << std::endl;

    // Send controller output
    output->SetValue(0, 0, att_control.x);
    output->SetValue(1, 0, att_control.y);
    output->SetValue(2, 0, att_control.z);
    output->SetValue(3, 0, force_thrust);
    output->SetDataTime(data->DataTime());

    ProcessUpdate(output);
}

void MyController::Reset(void)
{
    first_update = true;
}

void MyController::SetValues(Vector3Df pos_error, Vector3Df vel_error, Quaternion currentQuaternion, Vector3Df omega)
{
    // Define variables used in the controller. 
    input->SetValue(0, 0, pos_error.x);
    input->SetValue(1, 0, pos_error.y);
    input->SetValue(2, 0, pos_error.z);

    input->SetValue(0, 1, vel_error.x);
    input->SetValue(1, 1, vel_error.y);
    input->SetValue(2, 1, vel_error.z);

    input->SetValue(0, 2, currentQuaternion.q0);
    input->SetValue(1, 2, currentQuaternion.q1);
    input->SetValue(2, 2, currentQuaternion.q2);
    input->SetValue(3, 2, currentQuaternion.q3);

    input->SetValue(0, 3, omega.x);
    input->SetValue(1, 3, omega.y);
    input->SetValue(2, 3, omega.z);
}

void MyController::Saturate(Vector3Df &vec, Vector3Df sat)
{
    if (vec.x > sat.x)
    {
        vec.x = sat.x;
    }
    else if (vec.x < -sat.x)
    {
        vec.x = -sat.x;
    }

    if (vec.y > sat.y)
    {
        vec.y = sat.y;
    }
    else if (vec.y < -sat.y)
    {
        vec.y = -sat.y;
    }

    if (vec.z > sat.z)
    {
        vec.z = sat.z;
    }
    else if (vec.z < -sat.z)
    {
        vec.z = -sat.z;
    }
}