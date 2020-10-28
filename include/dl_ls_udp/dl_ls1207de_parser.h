#ifndef dl_LS1207DE_PARSER__
#define dl_LS1207DE_PARSER__

#include <dl_ls_udp/parser_base.h>

namespace dl_ls_udp
{
class CdlLs1207DEParser : public CParserBase
{
public:
    CdlLs1207DEParser();
    virtual ~CdlLs1207DEParser();

    virtual int Parse(char *data, size_t data_length, dlLsConfig &config, sensor_msgs::LaserScan &msg);

    void SetRangeMin(float minRange);
    void SetRangeMax(float maxRange);
    void SetTimeIncrement(float time);
    void SetAngleResolution(float resolution);    
    void SetFrameId(std::string frame_id);

private:
    float fRangeMin;
    float fRangeMax;
    float fTimeIncrement;
    float fangleResolution;
    int   mSamplePointNum;        
    std::string fFrame_id;
};
} /*namespace dl_ls_udp*/

#endif /*dl_LS1207DE_PARSER__*/
