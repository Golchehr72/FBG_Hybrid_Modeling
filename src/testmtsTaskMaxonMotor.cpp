//qt
#include <QApplication>

//ROS
#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
//#include <cisst_ros_crtk/mts_ros_crtk_bridge.h>

//Maxon
#include <maxonControl/maxonMotor.h>
#include <maxonControl/maxonInterface.h>
//#include <maxonControl/maxonMotorInterface.h>

//CISST
#include <cisstMultiTask/mtsManagerLocal.h>
//#include <cisstOSAbstraction/osaGetTime.h>
//#include <cisstMultiTask/mtsCollectorState.h>
//#include <cisstMultiTask/mtsCollectorQtComponent.h>

//Maxon UI
#include <maxonUI/maxonWidget.h>
#include <maxonUI/maxonStatusWidget.h>
#include <maxonUI/maxonControlWidget.h>

int main(int argc, char *argv[])
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);

    //Path to config files
    std::string path_to_config_dir = "/home/golchehr/bigss/catkin_ws/src/snake/config/"; //Maxon Motor Config file
    // std:: string motor_json="/home/golchehr/bigss/catkin_ws/src/snake/config/maxonSetupOldBenchtopTwoCableBend.json"
    //std::string ur_program = "/home/bigss/bigss/util/universalRobot/ur_programs/BIGSSURServer.ur"; UR

    //ROS Setup
    ros::init(argc, argv, "MotorController", ros::init_options::AnonymousName);
    ros::NodeHandle rosNodeHandle;
 
    QApplication app(argc, argv); //Start QT App

    mtsComponentManager * manager = mtsManagerLocal::GetInstance();

    //ROS Bridge Setup
    mtsROSBridge* rosBridge = new mtsROSBridge("bridge", 10.0 * cmn_ms, &rosNodeHandle); //mtsROSBridge bridge("publisher", Period in seconds(cmn_ms=10^-3), &rosNodeHandle);
    rosBridge->PerformsSpin(true); //bridge.PerformsSpin(true);
    manager->AddComponent(rosBridge);

    //Maxon Mototr
    //std::vector<std::string> motor_names={"bendleft","bendright"}; //NodeID1:motorTask=bendright, NodeID3:motorTask=bendleft
    std::vector<std::string> motor_names;
    maxonInterface mi("maxonInterface"); // this only initializes the mtsComponent class
    std::cout << "adding maxon interface" <<std::endl;
    std::cout << "adding maxon nodes" <<std::endl;
    bool maxonsConnected = mi.initialize(); // this sets the controller handle for each of the motor tasks
    CMN_LOG_INIT_DEBUG << "adding maxon nodes" << std::endl;
    mi.addNodes(path_to_config_dir+"maxonSetupOldBenchtopTwoCableBend.json"); // this creates the motor tasks & interfaces per config file
    //mi.addNodes(motor_json);

    //Maxon Control Widget,Add component to mtsComponentManager
    /*maxonWidget mw(&mi)
    manager->AddComponent(mw.statusWidget)
    */

    //Maxon Control, Add component to mtsComponentManager
    // maxonWidget mw(&mi);
    //manager->AddComponent(&mi);

    //Maxon Control Widget, Adding each motor task to mtsComponentManager
    int numMotors = mi.getNumMotors(); //Output is the number of motors
    std::cout << "Number of motors is: "<< numMotors << std::endl;
    for (int i=0 ; i < numMotors ; i++)
    {
        maxonMotor *m = mi.getMotor(i);
        CMN_LOG_RUN_VERBOSE << "Adding Motor " << m->GetName() << "To manager" << std::endl;
        manager->AddComponent(m);
        //m->clearFault();
        mtsBool enabled;
        m->enable(enabled);
        m->setPositionMode();
        //manager->AddComponent(mw.controls[i]);
        motor_names.push_back(m->GetName());
    }
    


    //Maxon Control Widget, Adding each motor task to mtsComponentManager
    /*int numMotors = mi.getNumMotors(); //Output is the number of motors
    for (int i = 0 ; i < numMotors ; i++)
    {
        maxonMotor *m = mi.getMotor(i);  //Output is motor task
        std::cout << "Adding motor " << m->GetName() << " to manager " << std::endl;
        manager->AddComponent(m);

    }
    */


     //Creat ROS Publisher and Subscribe for Maxon Motors
    std::cout<<"Adding publisher and subscribers"<<std::endl;
    /*
    rosBridge->AddPublisherFromCommandRead  
    <double, std_msgs::Float32>
    (motor_name +"_required_interface_state_PM","position",
    "/motor_"+motor_name+"/measured_jp");
    */
    // std::string motor_name;
    // for (int i=0 ; i < 1 ; i++){
    //     motor_name = motor_names[i];
    //     std::cout<<"motor name is:"<<motor_name<<std::endl;
    for(auto& motor_name : motor_names){
        //Position Measurement
        rosBridge->AddPublisherFromCommandRead                         //Publish maxon motor position to ROS
        <double, std_msgs::Float32>                                    //example: <vctDoubleVec, cisst_msgs::vctDoubleVec>
        (motor_name +"_required_interface_state_position","position",  //interface required name, //Function Name, it can be found in maxonMotorinterface.cpp -> Configurecontrolinterface
        motor_name+"/measured_position_jp");                           //Topic name

        //Velocity Measurement
        rosBridge->AddPublisherFromCommandRead
        <double, std_msgs::Float32>
        (motor_name +"_required_interface_state_velocity","velocity",
        motor_name+"/measured_velocity_jv");

        //Current Measurement
        //rosBridge->AddPublisherFromCommandRead  // mesuare current
        //<short, std_msgs::Int32>
        //(motor_name +"_required_interface_state_current","current",
        //motor_name+"/measured_current_jc");
        
        //Send absolute position commands to ROS
        //rosBridge->AddSubscriberToCommandWrite  
        //<double, std_msgs::Float32>
        //(motor_name +"_required_interface_control_set_abs_position","moveToAbsolutePosition",
        //motor_name+"/set_abs_position_jp");

        //Send relative position commands to ROS
        rosBridge->AddSubscriberToCommandWrite  
        <double, std_msgs::Float32>
        (motor_name +"_required_interface_control_set_rel_position","moveToRelativePosition",
        motor_name+"/set_rel_position_jp");

        //Send velocity commands to ROS
        rosBridge->AddSubscriberToCommandWrite 
        <double, std_msgs::Float32>
        (motor_name +"_required_interface_control_set_velocity","setVelocity",
        motor_name+"/set_velocity_jv");

        //Stop command to ROS
        //rosBridge->AddSubscriberToCommandVoid  
        //(motor_name +"_required_interface_control_stop","stop",
        //motor_name+"/stop_js");

        //Publisher
        manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_state_position",motor_name,"state");
        manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_state_velocity",motor_name,"state");
        //manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_state_current",motor_name,"state");
        //Subscriber
        manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_control_set_rel_position",motor_name,"control");
        //manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_control_set_abs_position",motor_name,"control");
        manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_control_set_velocity",motor_name,"control");
        //manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_control_stop",motor_name,"control");
    }

    std::cout<<"Ros Topics have been created"<<std::endl;

    //mw.Connect(manager);
  

    manager->CreateAll();
    manager->StartAll();

    //std::cout<<"Out2"<<std::endl;
    ros::spin();
    
}
    







/*

    for ( auto& motor_name : motor_names)
    {
        std::cout<<"In the Loop"<<std::endl;
        rosBridge -> AddPublisherFromCommandRead //Publish maxon motor position to ROS
        <double, std_msgs::Float32> // example: <vctDoubleVec, cisst_msgs::vctDoubleVec>
        (motor_name+"_required_interface_state", //interface required name 
        "position", //Function Name, it can be found in maxonMotorinterface.cpp -> ConfigureSatateinterface
        "/motor_" + motor_name + "/measured_jp"); //Topic name

        rosBridge -> AddSubscriberToCommandWrite //Subscribe maxon motor absolute position to ROS
        <double , std_msgs :: Float32>
        (motor_name + "_required_interace_control", //interface required name 
        "moveToAbsolutePosition", //Function Name, it can be found in maxonMotorinterface.cpp -> Configurecontrolinterface
        "/motor_"+ motor_name + "/move_jp"); //Topic name


        manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_state",motor_name,"state");
        manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_control",motor_name,"control");

    }
    */
    

    //Data Record
    /*
    mi.createCollectors();
    mi.addCollectors(manager);
    */


    //Maxon Control Initialization
    //mi.Connect(manager);

    
    /*
    for(int i = 0; i < numMotors ; i++)
    {
        maxonMotor *m = mi.getMotor(i); //Output is motor task
        m->clearFault();
        mtsBool enabled; 
        m->enable(enabled);
        m->setPositionMode();
        std::cout<<"Loop"<<std::endl;
    }
    */
    



/*
    for (auto& motor_name : motor_names)
    {
    motor_name->clearFault();
    motor_name->clearFault();
    mtsBool motor_enabled; 
    motor_name->enable(motor_enabled);
    motor_name->setPositionMode();
    }
    */


    //Maxon Control Widget Initialization
    /*
     mw.Connect(manager);

    manager->CreateAll();
    manager->StartAll();


    mw.show();
    app.exec();
    */


