#ifndef MTSTASKFBGINTERROGATOR_H
#define MTSTASKFBGINTERROGATOR_H

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <fbgInterrogator/interrogator.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsGenericObjectProxy.h>


class mtsTaskFBGInterrogator: public mtsTaskPeriodic{
    
    // CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);
    
    public:
    interrogator fbg_it;

    mtsTaskFBGInterrogator(const std::string& taskname = "FBGInterrogator", const double period=100.0*cmn_ms, const std::string& ip_address="192.168.1.11", const int port = 1852, const std::vector<int>& channels={});
    ~mtsTaskFBGInterrogator(void);
    void Startup(void){};
    void Run(void);
    void Cleanup(void){};


    private:
    std::vector<int> channels_used;
    std::vector<vctDoubleVec> peaks_raw; 
    vctDoubleVec peak_values; // (num_nodes_per_fiber * num_fibers) x 1 
    vctDoubleVec peak_values_no_common_mode_fiber; // The common mode from each fiber is removed (Iordachita et al Int J CARS 2009 A sub-milimetric...)
    vctDoubleVec peak_values_no_common_mode_node_loc; // The common mode from each node at the same location on the substrate is removed

    size_t num_fibers;
    size_t num_nodes;

    void InitializeVectorSizes();
    void ReadPeaksRawFromInterrogator();

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTaskFBGInterrogator);

#endif // MTSTaskFBGINTERROGATOR_H
