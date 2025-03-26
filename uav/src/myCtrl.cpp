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

    // Matrix descriptor for logging. It should be always a nx1 matrix. 
    MatrixDescriptor *log_labels = new MatrixDescriptor(3, 1);
    log_labels->SetElementName(0, 0, "x_error");
    log_labels->SetElementName(1, 0, "y_error");
    log_labels->SetElementName(2, 0, "yaw_error");
    state = new Matrix(this, log_labels, floatType, name);
    delete log_labels;

    // GUI for custom PID
    GroupBox *gui_customPID = new GroupBox(position, name);
    GroupBox *general_parameters = new GroupBox(gui_customPID->NewRow(), "General parameters");
    deltaT_custom = new DoubleSpinBox(general_parameters->NewRow(), "Custom dt [s]", 0, 1, 0.001, 4);
    mass = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Mass [kg]", 0, 10, 0.01, 3);
    k_motor = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Motor constant", 0, 10, 0.01, 3);
    sat_pos = new DoubleSpinBox(general_parameters->NewRow(), "Saturation pos", 0, 10, 0.01, 3);
    sat_att = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Saturation att", 0, 10, 0.01, 3);
    sat_thrust = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Saturation thrust", 0, 10, 0.01, 3);

    // Custom cartesian position controller
    GroupBox *custom_position = new GroupBox(gui_customPID->NewRow(), "Custom position controller");
    Kp_pos = new Vector3DSpinBox(custom_position->NewRow(), "Kp_pos", 0, 100, 0.1, 3);
    Kd_pos = new Vector3DSpinBox(custom_position->LastRowLastCol(), "Kd_pos", 0, 100, 0.1, 3);
    Ki_pos = new Vector3DSpinBox(custom_position->LastRowLastCol(), "Ki_pos", 0, 100, 0.1, 3);

    // Custom attitude controller
    GroupBox *custom_attitude = new GroupBox(gui_customPID->NewRow(), "Custom attitude controller");
    Kp_att = new Vector3DSpinBox(custom_attitude->NewRow(), "Kp_att", 0, 100, 0.1, 3);
    Kd_att = new Vector3DSpinBox(custom_attitude->LastRowLastCol(), "Kd_att", 0, 100, 0.1, 3);
    Ki_att = new Vector3DSpinBox(custom_attitude->LastRowLastCol(), "Ki_att", 0, 100, 0.1, 3);

    AddDataToLog(state);
}

MyController::~MyController()
{
    delete state;
}

void MyController::UpdateFrom(const io_data *data)
{
    float current_time = double(GetTime())/1000000000-initial_time;
    float thrust = 0.0;
    Vector3Df u, tau;

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

    // Get tunning parameters from GUI
    Vector3Df Kp_pos_val(Kp_pos->Value().x, Kp_pos->Value().y, Kp_pos->Value().z);
    Vector3Df Kd_pos_val(Kd_pos->Value().x, Kd_pos->Value().y, Kd_pos->Value().z);
    Vector3Df Ki_pos_val(Ki_pos->Value().x, Ki_pos->Value().y, Ki_pos->Value().z);
    Vector3Df Kp_att_val(Kp_att->Value().x, Kp_att->Value().y, Kp_att->Value().z);
    Vector3Df Kd_att_val(Kd_att->Value().x, Kd_att->Value().y, Kd_att->Value().z);
    Vector3Df Ki_att_val(Ki_att->Value().x, Ki_att->Value().y, Ki_att->Value().z);

    // Cartesian custom controller
    u.x = Kp_pos_val.x*pos_error.x + Kd_pos_val.x*vel_error.x;
    u.y = Kp_pos_val.y*pos_error.y + Kd_pos_val.y*vel_error.y;
    u.z = Kp_pos_val.z*pos_error.z + Kd_pos_val.z*vel_error.z;
    u.Saturate(sat_pos->Value());

    // Attitude custom controller
    Euler rpy = q.ToEuler();    
    tau.x = Kp_att_val.x*(rpy.roll + u.y) + Kd_att_val.x*omega.x;
    tau.y = Kp_att_val.y*(rpy.pitch - u.x) + Kd_att_val.y*omega.y;
    tau.z = Kp_att_val.z*(rpy.YawDistanceFrom(0)) + Kd_att_val.z*omega.z;
    applyMotorConstant(tau);
    tau.Saturate(sat_att->Value());

    // Compute custom thrust
    thrust = - u.z - mass->Value()*g;
    applyMotorConstant(thrust);
    thrust = thrust > sat_thrust->Value() ? sat_thrust->Value() : thrust;
    thrust = thrust < 0 ? 0 : thrust;   

    // Send controller output
    output->SetValue(0, 0, tau.x);
    output->SetValue(1, 0, tau.y);
    output->SetValue(2, 0, tau.z);
    output->SetValue(3, 0, thrust);
    output->SetDataTime(data->DataTime());

    // Log state (example). 
    // Modify the log_labels matrix in the constructor to add more variables.
    state->GetMutex();
    state->SetValue(0, 0, pos_error.x);
    state->SetValue(1, 0, pos_error.y);
    state->SetValue(2, 0, rpy.YawDistanceFrom(0));
    state->ReleaseMutex();

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

void MyController::applyMotorConstant(Vector3Df &signal)
{
    float motor_constant = k_motor->Value();
    signal.x = signal.x/motor_constant;
    signal.y = signal.y/motor_constant;
    signal.z = signal.z/motor_constant;
}

void MyController::applyMotorConstant(float &signal)
{
    float motor_constant = k_motor->Value();
    signal = signal/motor_constant;
}