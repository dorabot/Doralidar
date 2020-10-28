#ifndef dl_LS_COMMON_UDP__
#define dl_LS_COMMON_UDP__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dl_ls_common.h"

namespace dl_ls_udp
{
class CdlLsCommonUdp : public CdlLsCommon
{
public:
    CdlLsCommonUdp(const std::string &hostname, const std::string &port, int &timelimit, CParserBase *parser, ros::NodeHandle &nh, ros::NodeHandle &pnh);
    virtual ~CdlLsCommonUdp();

protected:
    /*Override functions*/
    virtual int InitDevice();
    virtual int CloseDevice();

    virtual int SendDeviceReq(const char *req, std::vector<unsigned char> *resp);

    virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize, int *length);

    int SendUdpData2Device(char *buf, int length);
private:
    int                          mSocket;
    size_t                       mBytesReceived;
    std::string                  mHostName;
    std::string                  mPort;
    int                          mTimeLimit;
    struct sockaddr_in           remoteServAddr;

};

} /*namespace dl_ls_udp*/

#endif /*dl_LS_COMMON_TCP__*/
