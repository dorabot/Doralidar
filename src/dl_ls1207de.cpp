#include <dl_ls_udp/dl_ls_common_udp.h>
#include <dl_ls_udp/dl_ls1207de_parser.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dl_ls1207de");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /*Check whether hostname is provided*/
    bool isTcpConnection = false;
    std::string strHostName;
    std::string strPort;
    if(pnh.getParam("hostname", strHostName))
    {
        isTcpConnection = true;
        pnh.param<std::string>("port", strPort, "2112");
    }

    /*Get configured time limit*/
    int iTimeLimit = 5;
    pnh.param("timelimit", iTimeLimit, 5);

    bool isDataSubscribed = false;
    pnh.param("subscribe_datagram", isDataSubscribed, false);

    int iDeviceNumber = 0;
    pnh.param("device_number", iDeviceNumber, 0);

    /*Create and initialize parser*/
    dl_ls_udp::CdlLs1207DEParser *pParser = new dl_ls_udp::CdlLs1207DEParser();
    double param;
    std::string frame_id;

    if(pnh.getParam("range_min", param))
    {
        ROS_INFO("range_min: %f", param);
        pParser->SetRangeMin(param);
    }
    if(pnh.getParam("range_max", param))
    {
        ROS_INFO("range_max: %f", param);
        pParser->SetRangeMax(param);
    }
    if(pnh.getParam("time_increment", param))
    {
        ROS_INFO("time_increment: %f", param);
        pParser->SetTimeIncrement(param);
    }

    if(pnh.getParam("angle_resolution", param))
    {
        ROS_INFO("angle_resolution: %f", param);
        pParser->SetAngleResolution(param);
    }		

    if(pnh.getParam("frame_id", frame_id))
    {
        ROS_INFO("frame_id: %s", frame_id.c_str());
        pParser->SetFrameId(frame_id);
    }


    /*Setup UDP connection and attempt to connect/reconnect*/
    dl_ls_udp::CdlLsCommon *pdlLs = NULL;
    int result = dl_ls_udp::ExitError;
    while(ros::ok())
    {
        if(pdlLs != NULL)
        {
            delete pdlLs;
        }

        pdlLs = new dl_ls_udp::CdlLsCommonUdp(strHostName, strPort, iTimeLimit, pParser, nh, pnh);
        result = pdlLs->Init();

        /*Device has been initliazed successfully*/
        while(ros::ok() && (result == dl_ls_udp::ExitSuccess))
        {
            ros::spinOnce();
            result = pdlLs->LoopOnce();
        }

        if(result == dl_ls_udp::ExitFatal)
        {
            return result;
        }
    }

    if(pdlLs != NULL)
    {
        delete pdlLs;
    }

    if(pParser != NULL)
    {
        delete pParser;
    }

    return result;
}
