#include <dl_ls_udp/dl_ls_sensor_frame.h>
#include <dl_ls_udp/dl_ls_constants.h>
#include <string.h>

namespace dl_ls_udp
{
CdlLsSensFrame::CdlLsSensFrame()
{
    mSensDataLength = 0;
    m_pSensData  = NULL;
    mAngleResolution = 0.5;
    mNumSampePoint = 270 * 2 + 1;
    mHigePrecisionFlag = false;		
}

CdlLsSensFrame::~CdlLsSensFrame()
{
    if(m_pSensData != NULL)
    {
        delete m_pSensData;
    }
}

uint8_t CdlLsSensFrame::GetFrameHeader()
{
    return m_pSensData->header;
}

uint8_t CdlLsSensFrame::GetCommandId()
{
    return m_pSensData->cmd_id;
}

uint16_t CdlLsSensFrame::GetRangeStart()
{
    return m_pSensData->range_start;
}

uint16_t CdlLsSensFrame::GetRangeEnd()
{
    return m_pSensData->range_end;
}

int  CdlLsSensFrame::GetSensDataCount()
{
    return m_pSensData->range_end - m_pSensData->range_start + 1;
}

uint16_t CdlLsSensFrame::GetSensDataOfIndex(int index)
{
    if(index < 0 || index > (m_pSensData->range_end - m_pSensData->range_start))
    {
        ROS_ERROR("Fail to get of index %d.", index);
        return 0;
    }

    return m_pSensData->sens_data[index];
}

uint16_t CdlLsSensFrame::GetSensIntensityOfIndex(int index)
{
    uint16_t offsetAddr = m_pSensData->range_end - m_pSensData->range_start + 1 + index;

    if(index < 0 || index > (m_pSensData->range_end - m_pSensData->range_start))
    {
        ROS_ERROR("Fail to get of index %d.", index);
        return 0;
    }

    return m_pSensData->sens_data[offsetAddr];
}

bool CdlLsSensFrame::CheckFrame(char *buff, int length, uint8_t value)
{
    return true;
}

void CdlLsSensFrame::SetAngleResolution(float angle_resolution)
{
    mAngleResolution = angle_resolution;
}

float CdlLsSensFrame::GetAngleResolution(void)
{
    return mAngleResolution;
}

void CdlLsSensFrame::SetNumSampePoint(int num)
{
    mNumSampePoint = num;
}

bool CdlLsSensFrame::GetIntensityEnable(void)
{
    return mIntensityEnable;
}

bool CdlLsSensFrame::GetHigePrecisionFlag(void)
{
    return mHigePrecisionFlag;
}

uint32_t CdlLsSensFrame::GetSampleTime_Sec(void)
{
    return this->mSample_Sec;
}

uint32_t CdlLsSensFrame::GetSampleTime_nSec(void)
{
    return this->mSample_nSec;
}

void CdlLsSensFrame::StrToTime(uint8_t *buf)
{
	struct tm tmp_time;
	uint8_t tmp_buf[32];
	uint32_t str_len = 0,k,usecend;
	
	char *pt_src = (char*)(buf + 1);
	char *pt_end = strchr(pt_src,',');
	char *pt_us = pt_end + 1;

	if(pt_src && pt_end)
	{
		str_len = pt_end - pt_src;

		if(str_len == 14)
		{
			for(k = 0;k< str_len;k++)
			{
				tmp_buf[k] = pt_src[k];
			}
			
			tmp_buf[k] = 0;	//add end 0
		}
		else
		{
			return;
		}
	}
	else
	{
		return ;
	}
	
	strptime((char*)tmp_buf,"%Y%m%d%H%M%S",&tmp_time); //����??24����??
	this->mSample_Sec = (uint32_t)mktime(&tmp_time);

	usecend = 0;
	for(k=0;k<6;k++)
	{	
		usecend *= 10;
		usecend += *(pt_us+k) - '0';
	}

	this->mSample_nSec = usecend * 1000;	
}



bool CdlLsSensFrame::InitFromSensBuff(char *buff, int length)
{
    int index = 0;
    uint16_t *tmp_buf = (uint16_t *)buff;
	
    if(buff == NULL)
    {
        ROS_ERROR("Invalide input buffer!");
        return false;
    }

    char *pData = new char[length + 100];
    if(pData == NULL)
    {
        ROS_ERROR("Insufficiant memory!");
        return NULL;
    }

    //memcpy(pData, buff, length);
    m_pSensData = new(pData) CdlLsSensFrame::SensData;
    mSensDataLength = length;

    if(mSensDataLength == mNumSampePoint * 2)
    {
        mIntensityEnable = false;
        mAddInfoEnable = false;
    }
    else if(mSensDataLength == mNumSampePoint * 2 * 2)
    {
        mIntensityEnable = true;
        mAddInfoEnable = false;
    }
    else if(mSensDataLength == mNumSampePoint * 2 * 2 + 128)
    {
        mIntensityEnable = true;
        mAddInfoEnable = true;
    }
    else
    {
        mIntensityEnable = false;
        mAddInfoEnable = false;

        ROS_ERROR("config angle_resolution in launch file err !");
        return NULL;
    }	

    /*System is using LOW END ENCODING, swtich the words*/
    m_pSensData->range_start = 0; //SWITCH_UINT16(m_pSensData->range_start);
    m_pSensData->range_end   = mNumSampePoint - 1;; //SWITCH_UINT16(m_pSensData->range_end);

    /*Switch sensor data*/
    if(mAddInfoEnable)
    {
        for(int k = 0; k < 128; k++)
        {
            m_pSensData->add_info[k] = buff[k];
        }

        if(m_pSensData->add_info[66])
        {
            mHigePrecisionFlag = true;
        }
        else
        {
            mHigePrecisionFlag = false;
        }

		//uint8_t *pt_time_str = m_pSensData->add_info + 69; 
		//this->StrToTime(pt_time_str);

        tmp_buf = (uint16_t *)(buff + 128);
    }
    else
    {
        mHigePrecisionFlag = false;
        tmp_buf = (uint16_t *)buff;
    }

	//mHigePrecisionFlag = false;

    index = 0;
    while(index < mNumSampePoint)
    {
        m_pSensData->sens_data[index] = SWITCH_UINT16(tmp_buf[index]);
        index ++;
    }

    if(mIntensityEnable)
    {
        while(index < mNumSampePoint * 2)
        {
            m_pSensData->sens_data[index] = SWITCH_UINT16(tmp_buf[index]);
            index ++;
        }
    }

    return true;
}

void CdlLsSensFrame::DumpFrameHeader()
{
    if(m_pSensData == NULL || mSensDataLength == 0)
    {
        return;
    }

    ROS_DEBUG("Frame Header: 0x%02X", this->GetFrameHeader());
    ROS_DEBUG("Command   ID: 0x%02X", this->GetCommandId());
    ROS_DEBUG("Angle  START: 0x%04X", this->GetRangeStart());
    ROS_DEBUG("Angle    END: 0x%04X", this->GetRangeEnd());
}

void CdlLsSensFrame::DumpFrameData()
{
    if(m_pSensData == NULL || mSensDataLength == 0)
    {
        return;
    }

    int dataCount = this->GetSensDataCount();
    ROS_DEBUG("Data   Count: %d", dataCount);

    int idx = 1;
    while(idx <= dataCount)
    {
        printf("%u ", static_cast<unsigned int>(this->GetSensDataOfIndex(idx - 1)));

        idx++;
        if(idx % 48 == 0)
        {
            printf("\n");
        }
    }
    printf("\n");
}

} /*namespace dl_ls_udp*/


