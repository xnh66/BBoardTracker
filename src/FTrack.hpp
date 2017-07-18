#ifndef FTRACK_HPP
#define FTRACK_HPP

//#define FOR_SHOW
//#define SKIP_BF

#include "AR_utils.hpp"

#include "ORBextractor.hpp"

#include <unistd.h>
#include<algorithm>
#include<chrono>




class FTrack{
    //class TermCriteria termcrit_t1(1,30,0.03);
public:

    bool Init(const Mat& _frame, const Quadrangle& _qua);
    int Process(const Mat& _frame);
    void GetQua(Quadrangle& _qua);
    int BF(const Mat& _frame);
    bool isInPicture(Rect& _rect);
    Point2f GetCenter(Quadrangle& _qua);
    Quadrangle GetNewQua(Point2f& _center,int _w, int _h);
    void GetFinalPoint(Point2f& _FP);
    Point2f GetCentroid(vector<Point2f>& _Points);
    int GetStatus();

    FTrack();

    Point2f FinalPoint;     //Anchor point

private:

    // constant
    int maxPtsNum;

    // max tolerance for BF
    int maxToleranceBFNum;

    //picture size
    int PW;
    int PH;

    // whether skip BF or not
    int forBFstatus;

    //for update
    int NofCQ;
    Mat img_old,img_new;
    Mat gray_old,gray_new;
    Quadrangle mInitQua,mIQua,tmpQua,mCQua,tmpCQua; //CQua-CenterQua
    vector<Point2f> points_old,points_new,points_centerold,points_centernew;
    vector<uchar> maskOF,maskBC,maskC,nmaskC;    //C means Center ;B means for BF;n means new

    //track status
    int status;
    int FailCount;

    //for full-lost BF
    vector<KeyPoint> kps1;
    Mat des1;
    int TH_LOW=60;
    int TH_HIGH;
    int HISTO_LENGTH=30;
    float NNratio=1.0;
    bool mbCheckOrientation =false;
    int ThRANSAC =6;
    vector<int> rotHist[30];  //30
    const float factor = 1.0f/30;

    //for KeyFrame
    int KFcount;
    int KFfirst;
    float KFSSD1,KFSSD2;
    Quadrangle KFQua1,KFQua2;



};

#endif // FTRACK_HPP
