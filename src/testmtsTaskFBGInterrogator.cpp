
#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <fbg_hybrid_modeling/mtsTaskFBGInterrogator.h>
#include <std_msgs/Float64MultiArray.h>

// NOTE: To use this code, you need to be connected to the interrogator via ethernet and give your ethernet port the address 192.168.1.11
// use the command ```sudo ip address add 192.168.1.10/24 dev enpxxx```, where enpxxx is the name of your ethernet port, which you can find 
// via the command ```ip address list```

int main (int argc, char *argv[]){
    std::cout << "begin" << std::endl;

    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    ros::init(argc, argv, "FBGInterrogator", ros::init_options::AnonymousName);
    ros::NodeHandle rosNodeHandle;
    
    mtsComponentManager * manager = mtsManagerLocal::GetInstance();
    mtsROSBridge* rosBridge = new mtsROSBridge("fbg_interrogator_bridge", 10.0 * cmn_ms, &rosNodeHandle);
    manager->AddComponent(rosBridge);
    // rosBridge->PerformsSpin(true);
    mtsTaskFBGInterrogator* fbg_interrogator = new mtsTaskFBGInterrogator("fbg_interrogator", 10*cmn_ms, "192.168.1.11", 1852, {1,2}); // TODO: Default args not working 

    manager->AddComponent(fbg_interrogator);
    std::cout << "begin ros bridge" << std::endl;
    rosBridge->AddPublisherFromCommandRead  // send velocity commands to ROS
    <vctDoubleVec, std_msgs::Float64MultiArray>
    ("fbg_interrogator_required", "peak_values",
    "/fbg_interrogator/peak_values");

    rosBridge->AddPublisherFromCommandRead  // send velocity commands to ROS
    <vctDoubleVec, std_msgs::Float64MultiArray>
    ("fbg_interrogator_required", "peak_values_no_common_mode_fiber",
    "/fbg_interrogator/peak_values_no_common_mode_fiber");
    std::cout << "end ros bridge" << std::endl;

    manager->Connect(rosBridge->GetName(), "fbg_interrogator_required", fbg_interrogator->GetName(), "fbg_interrogator_provided");
    manager->CreateAllAndWait(5.0*cmn_ms);
    manager->StartAllAndWait(5.0*cmn_ms);
      
    ros::spin();


}
