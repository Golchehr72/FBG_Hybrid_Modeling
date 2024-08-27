
#include <fbg_hybrid_modeling/mtsTaskFBGInterrogator.h>
#include <ros/ros.h>

#include <cisst_ros_bridge/mtsROSBridge.h>
#include <fbgInterrogator/interrogator.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>


#include <unistd.h>

// Instantiates BIGSS util interrogator class and publishes peaks to ROS


mtsTaskFBGInterrogator::mtsTaskFBGInterrogator(const std::string& taskname, const double period, const std::string& ip_address, const int port, const std::vector<int>& channels):
  mtsTaskPeriodic(taskname, period),
  channels_used(channels){
    if (!fbg_it.connect(ip_address, port)) {
        std::cout << "Could not connect to FBG interrogator at " << ip_address << std::endl;
    }

    StateTable.AddData(peak_values, "peak_values");
    StateTable.AddData(peak_values_no_common_mode_fiber, "peak_values_no_common_mode_fiber");

    mtsInterfaceProvided *provided = AddInterfaceProvided("fbg_interrogator_provided");
    if(provided) {
      provided->AddCommandReadState(StateTable, peak_values, "peak_values");
      provided->AddCommandReadState(StateTable, peak_values_no_common_mode_fiber, "peak_values_no_common_mode_fiber");
      provided->AddCommandReadState(StateTable, StateTable.PeriodStats, "GetPeriodStatistics");
    }
}

mtsTaskFBGInterrogator::~mtsTaskFBGInterrogator(){}

void mtsTaskFBGInterrogator::Run(){
  ProcessQueuedCommands(); // Needed for cisst mts to run
  ProcessQueuedEvents();

  if(!num_fibers){ // initialize members to correct size based on data
    InitializeVectorSizes();
  }
  ReadPeaksRawFromInterrogator(); // updates peaks_raw

  for(size_t i : channels_used){ 

    double avg_common_mode_ith_fiber = peaks_raw[i].SumOfElements()/num_nodes;

    for(size_t j=0; j<num_nodes; j++){

      // to get ref wavelengths: collect 200x peak_values and average across 200 elements

      // ref_wavelength = [a b c d e f g h i]
      // peak_values -= ref_wave_lengths
      peak_values[i*num_nodes + j] = peaks_raw[i](j); // fill in from top to bottom, one fiber at a time
      peak_values_no_common_mode_fiber[i*num_nodes+j] = peak_values[i*num_nodes + j] - avg_common_mode_ith_fiber; //remove common mode for fiber
    }
  }
}

void mtsTaskFBGInterrogator::InitializeVectorSizes(){
    ReadPeaksRawFromInterrogator();
    num_fibers = peaks_raw.size();
    num_nodes = peaks_raw[1].size(); // TODO: fix, e.g. nodes per fiber  
    peak_values.resize(num_fibers*num_nodes);
    peak_values_no_common_mode_fiber.resize(num_fibers*num_nodes);

    if (!channels_used.size()){ //If user did not specify which channels to use, then just use them all
      for (size_t i=0; i<num_fibers; i++){
        channels_used.push_back(i);
      }
    }
}

void mtsTaskFBGInterrogator::ReadPeaksRawFromInterrogator(){
  if( fbg_it.isConnected()) {
    if (fbg_it.getDataUnbuffered()){ //if you don't do unbuffered you lose data in long experiments
      fbg_it.extractPeaks(); // tells interrogator to update its data member called 'peaks' with the current values
      peaks_raw = fbg_it.getPeaks(); // gets the current value of 'peaks' (does not update it, so extractPeaks first)
    }
  }
}
