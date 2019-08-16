#include <correction_receiver_ros/correction_receiver_ros.h>

using namespace std;
using namespace correction_receiver;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "correction_receiver_ros_node");
    ros::NodeHandle nh_;

    Config conf;
    ros::NodeHandle nh_priv("~");

    nh_priv.getParam("RTKTopic", conf.rtkTopic);
    nh_priv.getParam("RTKFrameID", conf.rtkFrameID);
    nh_priv.getParam("BroadcastServerIP", conf.broadcastServerIP);
    nh_priv.getParam("BroadcastServerPort", conf.broadcastServerPort);
    nh_priv.getParam("ReceiverLocalPort", conf.receiverLocalPort);
    nh_priv.getParam("AuthenticationKey", conf.authenticationKey);
    nh_priv.getParam("TimeoutReAuth", conf.timeoutReAuth);
    nh_priv.getParam("MaxTransferredDataSize", conf.maxTransferredDataSize);
    nh_priv.getParam("RTKServiceSerial", conf.rtkServiceSerial);
    nh_priv.getParam("SerialBaudRate", conf.serialBaudRate);
    nh_priv.getParam("SerialDataBits", conf.serialDataBits);
    nh_priv.getParam("SerialStopBits", conf.serialStopBits);
    nh_priv.getParam("SerialParity", conf.serialParity);

    CorrectionReceiver *ptrCorReceiver = new CorrectionReceiver(conf, nh_);
    
    while(ros::ok()) //watch dog
    {
        if(ptrCorReceiver == nullptr)
            ptrCorReceiver = new CorrectionReceiver(conf, nh_);
        
        ptrCorReceiver->run();
        
        if(ptrCorReceiver)
        {
            delete ptrCorReceiver;
            ptrCorReceiver = nullptr;
        }
    }

    return 0;
}
