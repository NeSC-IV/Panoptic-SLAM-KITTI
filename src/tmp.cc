#include "Tracking.h"

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "Optimizer.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "GeometricTools.h"

#include <iostream>

#include <mutex>
#include <chrono>


using namespace std;

namespace ORB_SLAM3
{


Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)
{
    mImGray = im;
    Kp_toBe_Filtered.clear();
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if (mSensor == System::MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }
    else if(mSensor == System::IMU_MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
        }
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
    }

    while(!isNewDetectedImgArrived())
    {
        usleep(1);
    }

    Panoptic_image panoptic_result = mpPanopticNet->GetResults();
    mCurrentFrame.setPanoptic(panoptic_result.objs);
    mCurrentFrame.set_panoptic_masks(panoptic_result.all_masks);
    mPanopticImg = mpPanopticNet->pImg;
    mUnkImg = mpPanopticNet->unkImg;

    cv::Mat people_mask,mask_temp;
    people_mask = cv::Mat::zeros(cv::Size(1226, 370), CV_8UC3);

    int n_people = 0;
    
    for(int i=0; i<panoptic_result.objs.size(); i++){
     
        // removing people's keypoints a priori
        if(panoptic_result.objs[i].category_id == 2){  
            cv::threshold( panoptic_result.objs[i].mask, mask_temp, 1, 1, cv::THRESH_BINARY );
            n_people++;
            people_mask |= mask_temp;

        }

    }

    mCurrentFrame.SetPanopticResults(people_mask);

    mCurrentFrame.RemoveOutliersMono(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    if(mState != NO_IMAGES_YET)
    {
        cv::Mat Static_F, mask_F;
        
        mpPanopticNet->ShortTerm_DA(mCurrentFrame.Panoptic_objs, mLastFrame.Panoptic_objs);
        curr_thing_keypoints.clear();
        curr_thing_descriptor.clear();
        Thing_Stuff_Detachment(); // fill curr_thing_keypoints, curr_thing_descriptor, mCurrentFrame.stuff_keypoints, mCurrentFrame.stuff_descriptor
        Unknown_Mask_Keypoints();
        Find_Stuff_Fundamental_Matrix(Static_F, mask_F);

        //Find_Whole_Fundamental_Matrix(Static_F, mask_F);
        if(!Static_F.empty())
        {  
            Find_Thing_Fundamental_Matrix(Static_F,mask_F);
            Find_Unknown_Fundamental_Matrix(Static_F);

            //update last thing content
            last_thing_keypoints.clear();
            last_thing_descriptor.clear();
            last_thing_keypoints.insert(curr_thing_keypoints.begin(),curr_thing_keypoints.end());
            last_thing_descriptor.insert(curr_thing_descriptor.begin(),curr_thing_descriptor.end());
        }
    }
    else
    {
        std::vector<Panoptic_Object> last_objs;
        mpPanopticNet->ShortTerm_DA(mCurrentFrame.Panoptic_objs,last_objs);
        Thing_Stuff_Detachment(); // fill curr_thing_keypoints, curr_thing_descriptor, mCurrentFrame.stuff_keypoints, mCurrentFrame.stuff_descriptor
        Unknown_Mask_Keypoints();
        
        last_thing_keypoints.insert(curr_thing_keypoints.begin(),curr_thing_keypoints.end());
        last_thing_descriptor.insert(curr_thing_descriptor.begin(),curr_thing_descriptor.end());
    }
    mCurrentFrame.Remove_Moving_KeyPointsMono(Kp_toBe_Filtered,mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);

    std::cout<< "Frame ID: " << mCurrentFrame.mnId << "\n";

    if (mState==NO_IMAGES_YET)
        t0=timestamp;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    lastID = mCurrentFrame.mnId;
    Track();

    return mCurrentFrame.GetPose();
}


void Frame::Remove_Moving_KeyPointsMono(std::vector<cv::KeyPoint> kps, const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth)
{
    if(mvKeys.empty())
        return;
    
    std::vector<cv::KeyPoint> _mvKeys;
    cv::Mat _mDescriptors;
    int counter_bool_key = 0;
    //std::cout<<"Remove_Moving_KeyPoints " << kps.size() << "\n";
    for (unsigned int i=0; i < mvKeys.size(); i++) // check all Keypoints
    {
        bool bool_key = false;
        cv::KeyPoint orb_kp = mvKeys[i];
        for (unsigned int j=0; j < kps.size(); j++)
        {
            cv::KeyPoint kp = kps[j];
            if(orb_kp.pt.x == kp.pt.x && orb_kp.pt.y==kp.pt.y)
            {
                bool_key = true;
            }
        }
        if(bool_key == false){      			
            _mvKeys.push_back(mvKeys[i]);
       	    _mDescriptors.push_back(mDescriptors.row(i));
        }
    }
    

    mvKeys = _mvKeys;
    mDescriptors = _mDescriptors;

    N = mvKeys.size();

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);
    mnCloseMPs = 0;

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mmProjectPoints.clear();// = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
    mmMatchedInImage.clear();

    mvbOutlier = vector<bool>(N,false);

    mVw.setZero();

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();
}


}