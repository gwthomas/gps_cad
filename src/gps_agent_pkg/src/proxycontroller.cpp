#include <sys/time.h>

#include "gps_agent_pkg/robotplugin.h"
#include "gps_agent_pkg/util.h"
#include "gps_agent_pkg/proxycontroller.h"

using namespace gps_control;

inline void print_ms(ros::Duration rt) {
    ROS_INFO("%f", rt.toSec() * 1000.0);
}

// Constructor.
ProxyController::ProxyController(ros::NodeHandle n)
: TrialController(), n(n)
{
    is_configured_ = false;
}

// Destructor.
ProxyController::~ProxyController() {
}

void ProxyController::get_action(int t, const Eigen::VectorXd &X, const Eigen::VectorXd &obs, Eigen::VectorXd &U){
    if (!is_configured_) {
        ROS_FATAL("Trying to get action, but not configured");
    }

    struct timeval t1, t2;
    gettimeofday(&t1, NULL);

    gps_agent_pkg::ProxyControl::Request request;
    gps_agent_pkg::ProxyControl::Response response;
    int dO = obs.size();
    request.obs.resize(dO);
    for (int i = 0; i < dO; ++i) {
        request.obs[i] = obs[i];
    }

    if (client_.call(request, response)) {
        int dU = response.action.size();
        U.resize(dU);
        for (int i = 0; i < dU; ++i) {
            U[i] = response.action[i];
        }
    } else {
        ROS_FATAL("Proxy control service call failed");
    }

    gettimeofday(&t2, NULL);
    double elapsed_time = (t2.tv_sec - t1.tv_sec) * 1000.0 + (t2.tv_usec - t1.tv_usec) / 1000.0;
    std::cout << "get_action took " << elapsed_time << " ms.\n";
}

// Configure the controller.
void ProxyController::configure_controller(OptionsMap &options)
{
    // std::string service_name = boost::get<std::string>(options["service_name"]);
    client_ = n.serviceClient<gps_agent_pkg::ProxyControl>("/proxy_control", true);
    ROS_INFO("Waiting for service...");
    client_.waitForExistence();
    ROS_INFO("Done");

    //Call superclass
    TrialController::configure_controller(options);
    ROS_INFO_STREAM("Set proxy parameters");
    is_configured_ = true;
}
