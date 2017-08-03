#pragma once

// Headers.
#include <vector>
#include <Eigen/Dense>

// Superclass.
#include "gps_agent_pkg/trialcontroller.h"
#include "gps_agent_pkg/ProxyControl.h"

namespace gps_control
{

    class ProxyController : public TrialController
    {
    private:
        ros::NodeHandle n;
        ros::ServiceClient client_;
    public:
        // Constructor.
        ProxyController(ros::NodeHandle n);
        // Destructor.
        virtual ~ProxyController();
        // Compute the action at the current time step.
        virtual void get_action(int t, const Eigen::VectorXd &X, const Eigen::VectorXd &obs, Eigen::VectorXd &U);
        // Configure the controller.
        virtual void configure_controller(OptionsMap &options);
    };

}
