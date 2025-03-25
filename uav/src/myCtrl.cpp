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
    // GroupBox *gui_customPD_position = new GroupBox(gui_customPD->NewRow(), "Position gains");
    // Kp_pos = new Vector3DSpinBox(gui_customPD_position->NewRow(), "Kp", 0, 20, 0.01, 3);
    // Kd_pos = new Vector3DSpinBox(gui_customPD_position->LastRowLastCol(), "Kd", 0, 10, 0.01, 3);
    // Ki_pos = new Vector3DSpinBox(gui_customPD_position->LastRowLastCol(), "Ki", 0, 5, 0.01, 3);
    // sat_pos = new Vector3DSpinBox(gui_customPD_position->LastRowLastCol(), "Saturation", 0, 10, 0.01, 3);
    mass = new DoubleSpinBox(gui_customPD->NewRow(), "Mass [kg]", 0, 10, 0.01, 3);
    // GroupBox *gui_customPD_attitude = new GroupBox(gui_customPD->NewRow(), "Attitude gains");
    // Kp_att = new Vector3DSpinBox(gui_customPD_attitude->NewRow(), "Kp", 0, 20, 0.01, 3);
    // Kd_att = new Vector3DSpinBox(gui_customPD_attitude->LastRowLastCol(), "Kd", 0, 10, 0.01, 3);
    // Ki_att = new Vector3DSpinBox(gui_customPD_attitude->LastRowLastCol(), "Ki", 0, 5, 0.01, 3);
    // sat_att = new Vector3DSpinBox(gui_customPD_attitude->LastRowLastCol(), "Saturation", 0, 10, 0.01, 3);


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

    // // Obtain gains for controller
    // Vector3Df Kpp(Kp_pos->Value().x, Kp_pos->Value().y, Kp_pos->Value().z);
    // Vector3Df Kdp(Kd_pos->Value().x, Kd_pos->Value().y, Kd_pos->Value().z);
    // Vector3Df Kip(Ki_pos->Value().x, Ki_pos->Value().y, Ki_pos->Value().z);

    // Vector3Df Kpa(Kp_att->Value().x, Kp_att->Value().y, Kp_att->Value().z);
    // Vector3Df Kda(Kd_att->Value().x, Kd_att->Value().y, Kd_att->Value().z);
    // Vector3Df Kia(Ki_att->Value().x, Ki_att->Value().y, Ki_att->Value().z);


    // Compute position controller
    // Vector3Df pos_control; 
    // pos_control.x = - Kpp.x*pos_error.x - Kdp.x*vel_error.x; //+ Ki_val.x*pos_error.x*delta_t;
    // pos_control.y = - Kpp.y*pos_error.y - Kdp.y*vel_error.y; //+ Ki_val.y*pos_error.y*delta_t;
    // pos_control.z = - Kpp.z*pos_error.z - Kdp.z*vel_error.z - mass->Value()*g; //+ Ki_val.z*pos_error.z*delta_t;
 
    // // Saturate control
    // pos_control.Saturate(sat_pos->Value());
    // // Saturate(pos_control, Vector3Df(sat_pos->Value().x, sat_pos->Value().y, sat_pos->Value().z));

    // // Compute attitude controller
    // Quaternion qt, qd;
    // // Quaternion qz(1, 0, 0, 0);
    // float force_thrust = - pos_control.GetNorm();
    // if (force_thrust > 0)
    // {
    //     force_thrust = 0;
    // }

    // if (pos_control.GetNorm() != 0)
    // {
    //     qt.q0 = DotProduct(Vector3Df(0, 0, -1), pos_control) + pos_control.GetNorm();
    //     Vector3Df cross = CrossProduct(Vector3Df(0, 0, -1), pos_control);
    //     qt.q1 = cross.x;
    //     qt.q2 = cross.y;
    //     qt.q3 = cross.z;
    //     qt.Normalize();
    //     qd = qt*qz;
    //     qd.Normalize();
    // }
    // else
    // {
    //     qt = q;
    //     qd = q;
    //     qd.Normalize();
    // }

    // Quaternion qe = qd.GetConjugate()*q;
    // Vector3Df thetae = 2*qe.GetLogarithm();
  
    // Vector3Df att_control;
    // att_control.x = Kpa.x*thetae.x + Kda.x*vel_error.x; //+ Ki_val.x*thetae.x*delta_t;
    // att_control.y = Kpa.y*thetae.y + Kda.y*vel_error.y; //+ Ki_val.y*thetae.y*delta_t;
    // att_control.z = Kpa.z*thetae.z + Kda.z*vel_error.z; //+ Ki_val.z*thetae.z*delta_t;

    // // Saturate control
    // att_control.Saturate(sat_att->Value());
    // // Saturate(att_control, Vector3Df(sat_att->Value().x, sat_att->Value().y, sat_att->Value().z));

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

    std::cout << "error xyz: " << pos_error.x << " " << pos_error.y << " " << pos_error.z << std::endl;

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

void MyController::SetValues(Vector3Df pos_error, Vector3Df vel_error, Quaternion currentQuaternion, Vector3Df omega, Quaternion qz)
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

    input->SetValue(0, 4, qz.q0);
    input->SetValue(1, 4, qz.q1);
    input->SetValue(2, 4, qz.q2);
    input->SetValue(3, 4, qz.q3);
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