//
//  AR_utils.cpp
//  SY_AR
//
//  Created by Xnh on 2017/1/5.
//  Copyright © 2017年 Xnh. All rights reserved.
//

#include "AR_utils.hpp"
//#include "opencv2/calib3d.hpp"

// get Image Quadrilateral from the given Rect
// the input Rect itself should be valid


Quadrangle GetQuadrfromRect(const Rect& rec){
    Quadrangle qua;
    qua.tl=Point2i(rec.tl());
    qua.tr=qua.tl+Point2f(rec.width,0.0);
    //qua.tl=qua.tr;
    qua.br=qua.tr+Point2f(0.0,rec.height);
    qua.bl=qua.tl+Point2f(0.0,rec.height);
    return qua;
}

// get minimal outter bound rectangle by the Quadrangle
// It should be guaranteed that the Quadrangle is at least partly inside the imgSize
Rect GetMBRfromQuadr(const Quadrangle& qua, const Size& imgSize){
    float a,b,c,d;
    a=min(min(qua.tl.x,qua.tr.x),min(qua.bl.x,qua.br.x));
    a=floor(a);
    a=(a>0)?a:0;
    b=min(min(qua.tl.y,qua.tr.y),min(qua.bl.y,qua.br.y));
    b=(b>0)?b:0;
    b=floor(b);
    c=max(max(qua.tl.x,qua.tr.x),max(qua.bl.x,qua.br.x));
    c=ceil(c);
    c=(c<imgSize.width)?c:imgSize.width;
    d=max(max(qua.tl.y,qua.tr.y),max(qua.bl.y,qua.br.y));
    d=ceil(d);
    d=(d<imgSize.height)?d:imgSize.height;
    return Rect(Point2f(a,b),Point2f(c,d));
}

vector<Point2f> KpsToP2fs(const vector<KeyPoint>& kps, const vector<uchar> mask){
    bool emptyMask=false;
    if(mask.empty())
        emptyMask=true;
    
    vector<Point2f> p2fs;
    for(int i=0;i<kps.size();i++){
        if(emptyMask)
            p2fs.push_back(kps[i].pt);
        else if(mask[i])
            p2fs.push_back(kps[i].pt);
    }
    return p2fs;
}

vector<Point2f> P2fsToP2fs(const vector<Point2f>& inP2fs, const vector<uchar> mask){
    
    // regard mask.emtpy()
    
    vector<Point2f> outP2fs(CountNonZero(mask));
    int k=0;
    for(int i=0;i<mask.size();i++){
        if(mask[i])
            outP2fs[k++]=inP2fs[i];
    }
    
    return outP2fs;
}

Quadrangle GetNewQua(const Quadrangle& inQua, Matx33f& H){
    Quadrangle outQua;
    Point3f tl_t = H*inQua.tl;  outQua.tl = Point2f(tl_t.x/tl_t.z,tl_t.y/tl_t.z);
    Point3f tr_t = H*inQua.tr;  outQua.tr = Point2f(tr_t.x/tr_t.z,tr_t.y/tr_t.z);
    Point3f bl_t = H*inQua.bl;  outQua.bl = Point2f(bl_t.x/bl_t.z,bl_t.y/bl_t.z);
    Point3f br_t = H*inQua.br;  outQua.br = Point2f(br_t.x/br_t.z,br_t.y/br_t.z);
    return outQua;
}

vector<uchar> OverlayMask(const vector<uchar>& originMask, const vector<uchar>& overlayer){
    vector<uchar> outMask=originMask;
    int j=0;
    for(int i=0;i<outMask.size();i++){
        if(outMask[i]){
            if(!overlayer[j])
                outMask[i]=0;
            j++;
        }
    }
    
    return outMask;
}

bool CheckOverlay(const vector<uchar>& originMask, const vector<uchar>& overlayedMask){
    if(originMask.size()!=overlayedMask.size())
        return false;
    for(int i=0;i<originMask.size();i++){
        if(overlayedMask[i] && !originMask[i])
            return false;
    }
    return true;
}


vector<Point2f> RemoveKMatrix(const vector<Point2f>& inP2fs, const Matx33f& K){
    vector<Point2f> outUPpts;
    outUPpts.clear();
    for(int i=0;i<inP2fs.size();i++){
        Point3f tmp=K.inv()*inP2fs[i];
        outUPpts.push_back(Point2f(tmp.x,tmp.y));
    }
    return outUPpts;
}


void GetCentrAndSqrR(const vector<Point2f>& p2fs, Point2f& centroid, float& sqrRadius){
    
    centroid = Point2f(0.0,0.0);
    sqrRadius=0.0;
    
    if(p2fs.size()==0)
        return;
    
    for(int i=0;i<p2fs.size();i++){
        centroid+=p2fs[i];
    }
    centroid = centroid*(1.0/p2fs.size());
    
    for(int i=0;i<p2fs.size();i++){
        Point2f tmp = p2fs[i] - centroid;
        float d = tmp.x*tmp.x+tmp.y*tmp.y;
        sqrRadius+=d;
    }
    sqrRadius = sqrRadius*(1.0/p2fs.size());
    
    return ;
}

Point2f GetCentr(const vector<Point2f>& p2fs, const vector<uchar> mask){
    
    bool emptyMask=false;
    if(mask.empty())
        emptyMask=true;
    
    Point2f centr = Point2f(0.0,0.0);
    if(p2fs.size()==0)
        return centr;
    int count =0;
    for(int i=0;i<p2fs.size();i++){
        if(emptyMask){
            centr+=p2fs[i];
            count++;}
        else if(mask[i]){
            centr+=p2fs[i];
            count++;
        }
    }
    
    centr = centr*(1.0/float(count));
    return centr;
}

bool IsInsideQuadrangle(const Quadrangle& qua, const Point2f& testPt){
    Point3f line0=CalcLineByTwoP2fs(qua.tl, qua.bl);
    Point3f line1=CalcLineByTwoP2fs(qua.bl, qua.br);
    Point3f line2=CalcLineByTwoP2fs(qua.br, qua.tr);
    Point3f line3=CalcLineByTwoP2fs(qua.tr, qua.tl);
    
    // homoPt.dot(line0)/sqrt(line0.x^2+line0.y^2) equals to distance from pt to line.
    Point3f homoPt=Point3f(testPt.x,testPt.y,1.0);
    if( homoPt.dot(line0) < 0 && homoPt.dot(line1) < 0 && homoPt.dot(line2) < 0 && homoPt.dot(line3)< 0 )
        return true;
    return false;
}

Point3f CalcLineByTwoP2fs(const Point2f& startP2f, const Point2f& endP2f){
    Point3f line=Point3f((startP2f.y-endP2f.y),(endP2f.x-startP2f.x),(startP2f.x*endP2f.y-endP2f.x*startP2f.y));
    return line;
}

float Det(Matx33f inM){
    float outDet=0.0;
    outDet =  inM(0,0)*(inM(1,1)*inM(2,2)-inM(1,2)*inM(2,1))
             +inM(0,1)*(inM(1,2)*inM(2,0)-inM(1,0)*inM(2,2))
             +inM(0,2)*(inM(1,0)*inM(2,1)-inM(1,1)*inM(2,0));
    return outDet;
}

float CalcQuaArea(const Quadrangle& qua){
    float area=0.0;
    float a,b,c,d;
    a=CalcPtDist(qua.tl,qua.bl);
    b=CalcPtDist(qua.bl,qua.br);
    c=CalcPtDist(qua.br,qua.tr);
    d=CalcPtDist(qua.tr,qua.tl);
    float x,y;
    x=CalcPtDist(qua.tl,qua.br);
    y=CalcPtDist(qua.tr,qua.bl);
    
    area=sqrt(4*x*x*y*y-(b*b+d*d-a*a-c*c)*(b*b+d*d-a*a-c*c))/4;
    return area;
}

float CalcPtDist(const Point2f& pt1, const Point2f&pt2){
    float ptDist=sqrt((pt1.x-pt2.x)*(pt1.x-pt2.x)+(pt1.y-pt2.y)*(pt1.y-pt2.y));
    return ptDist;
}


//  RMatrix decompose to euler angles
//  The rotation angle's order is 1.X, 2.Y, 3.Z
void RMatrixToEulerRadians_1X2Y3Z(const Matx33f& R, Vec3f& eulerRadians){
    float alpha = atan2(R(2,1),R(2,2));
    float beta = atan2(-R(2,0),sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
    float theta = atan2(R(1,0),R(0,0));
    
    eulerRadians=Vec3f(alpha, beta, theta);
    
    return;
}

Matx33f EulerToRx(float xRadians){
    Matx33f Rx(1,0,0,0,cos(xRadians),-sin(xRadians),0,sin(xRadians),cos(xRadians));
    return Rx;
}

Matx33f EulerToRy(float yRadians){
    Matx33f Ry(cos(yRadians),0,sin(yRadians),0,1,0,-sin(yRadians),0,cos(yRadians));
    return Ry;
}

Matx33f EulerToRz(float zRadians){
    Matx33f Rz(cos(zRadians),-sin(zRadians),0,sin(zRadians),cos(zRadians),0,0,0,1);
    return Rz;
}

Rect ExpandRectEdge(const Rect& recIn, const Size& sz, const int edge){
    //Rect recOut(recIn);
    Point2i tl = recIn.tl();
    Point2i br = recIn.br();
    
    tl.x = (recIn.tl().x-edge)<0? 0:(recIn.tl().x-edge);
    tl.y = (recIn.tl().y-edge)<0? 0:(recIn.tl().y-edge);
    br.x = (recIn.br().x+edge)>sz.width? sz.width:(recIn.br().x+edge);
    br.y = (recIn.br().y+edge)>sz.height? sz.height:(recIn.br().y+edge);
    
    Rect recOut=Rect(tl,br);
    return recOut;
}

int CountNonZero(const vector<uchar>& stats){
    int count=0;
    for(int i=0;i<stats.size();i++){
        if(!stats[i])
            continue;
        else
            count++;
    }
    return count;
}

float DistOfP2f(const Point2f& p2f_1, const Point2f& p2f_2){
    float dist;
    dist =sqrt( pow(p2f_1.x-p2f_2.x, 2.0) + pow(p2f_1.y-p2f_2.y, 2.0));
    return dist;
}


// copyed from ORB_SLAM2-ORBmatcher Class
int DistOfOrbFeature(const Mat& feature_1, const Mat& feature_2){
    
    const int *pa = feature_1.ptr<int32_t>();
    const int *pb = feature_2.ptr<int32_t>();
    
    int dist=0;
    
    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }
    
    return dist;
}

float CalcMatCond(const Matx33f& Hmat){
    Mat tmpMat=Mat(Hmat);
    Mat u,vt,w;
    SVDecomp(tmpMat, w, u, vt);
    
    float theta_max=w.at<float>(0,0);
    float theta_min=w.at<float>(2,0);
    /*
    float theta_mid=w.at<float>(1,0);
    if(!(theta_max > theta_min && theta_max > theta_mid && theta_mid > theta_min))
        cout<<"ALEART!"<<endl;
    else
        cout<<"RIGHT!"<<endl;
    */
    
    return theta_max/theta_min;
}

int ratioTest(vector<vector<cv::DMatch> > &matches, float _ratio)
{
    int removed = 0;
    // for all matches
    for ( vector<vector<cv::DMatch> >::iterator
         matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
    {
        // if 2 NN has been identified
        if (matchIterator->size() > 1)
        {
            // check distance ratio
            if ((*matchIterator)[0].distance / (*matchIterator)[1].distance > _ratio)
            {
                matchIterator->clear(); // remove match
                removed++;
            }
        }
        else
        { // does not have 2 neighbours
            matchIterator->clear(); // remove match
            removed++;
        }
    }
    return removed;
}

void symmetryTest( const std::vector<std::vector<cv::DMatch> >& matches1,
                  const std::vector<std::vector<cv::DMatch> >& matches2,
                  std::vector<cv::DMatch>& symMatches )
{
    
    // for all matches image 1 -> image 2
    for (std::vector<std::vector<cv::DMatch> >::const_iterator
         matchIterator1 = matches1.begin(); matchIterator1 != matches1.end(); ++matchIterator1)
    {
        
        // ignore deleted matches
        if (matchIterator1->empty() || matchIterator1->size() < 2)
            continue;
        
        // for all matches image 2 -> image 1
        for (std::vector<std::vector<cv::DMatch> >::const_iterator
             matchIterator2 = matches2.begin(); matchIterator2 != matches2.end(); ++matchIterator2)
        {
            // ignore deleted matches
            if (matchIterator2->empty() || matchIterator2->size() < 2)
                continue;
            
            // Match symmetry test
            if ((*matchIterator1)[0].queryIdx ==
                (*matchIterator2)[0].trainIdx &&
                (*matchIterator2)[0].queryIdx ==
                (*matchIterator1)[0].trainIdx)
            {
                // add symmetrical match
                symMatches.push_back(
                                     cv::DMatch((*matchIterator1)[0].queryIdx,
                                                (*matchIterator1)[0].trainIdx,
                                                (*matchIterator1)[0].distance));
                break; // next match in image 1 -> image 2
            }
        }
    }
    
}

vector<DMatch> OrbMatch(const Mat& trainDes, const Mat& queryDes, float threshold){
    vector<DMatch> matchResult(queryDes.rows);
    
    for(int i=0;i<queryDes.rows;i++){
        Mat queryDescriptor = queryDes.row(i);
        float minDis=256;
        int matchedIdx=-1;
        for(int j=0;j<trainDes.rows;j++){
            Mat trainDescritor = trainDes.row(j);
            float dis = DistOfOrbFeature(trainDescritor, queryDescriptor);
            if(dis<minDis){
                minDis=dis;
                matchedIdx=j;
            }
        }
        //if(matchedIdx!=-1)
        matchResult[i]=DMatch(i, matchedIdx, minDis);
        
    }
    
    return matchResult;
}

vector<vector<DMatch> > OrbMatch_2(const Mat& trainDes, const Mat& queryDes, float threshold){
    
    vector< vector<DMatch> > matchResult(queryDes.rows);
    
    for(int i=0;i<queryDes.rows;i++){
        Mat queryDescriptor = queryDes.row(i);
        float minDis=256;
        int matchedIdx=-1;
        for(int j=0;j<trainDes.rows;j++){
            Mat trainDescritor = trainDes.row(j);
            float dis = DistOfOrbFeature(trainDescritor, queryDescriptor);
            if(dis<minDis){
                minDis=dis;
                matchedIdx=j;
            }
        }
        //if(matchedIdx!=-1)
        matchResult[i].push_back(DMatch(i, matchedIdx, minDis));
        matchResult[i].push_back(DMatch(i, matchedIdx, minDis));
        
    }
    
    return matchResult;
    
}

// affineMat*qua_1 = qua_2
Matx23f AffineMatOfQuas(const Quadrangle& qua_1, const Quadrangle& qua_2){
    Matx23f affineMat;
    
    Mat b(8,1,CV_32FC1);
    b.at<float>(0,0)=qua_2.tl.x; b.at<float>(1,0)=qua_2.tl.y;
    b.at<float>(2,0)=qua_2.tr.x; b.at<float>(3,0)=qua_2.tr.y;
    b.at<float>(4,0)=qua_2.bl.x; b.at<float>(5,0)=qua_2.bl.y;
    b.at<float>(6,0)=qua_2.br.x; b.at<float>(7,0)=qua_2.br.y;
    
    vector<Point2f> quaP2fs(4);
    quaP2fs[0]=qua_1.tl;
    quaP2fs[1]=qua_1.tr;
    quaP2fs[2]=qua_1.bl;
    quaP2fs[3]=qua_1.br;
    
    Mat A=Mat::zeros(8,6,CV_32FC1);
    for(int i=0;i<4;i++){
        
        A.at<float>(2*i,0)=quaP2fs[i].x;
        A.at<float>(2*i,1)=quaP2fs[i].y;
        A.at<float>(2*i,2)=1.0;
        
        A.at<float>(2*i+1,3)=quaP2fs[i].x;
        A.at<float>(2*i+1,4)=quaP2fs[i].y;
        A.at<float>(2*i+1,5)=1.0;
    
    }
    
    Mat x(6,1,CV_32FC1);
    Mat AA = A.t()*A;
    x = AA.inv()*A.t()*b;
    for(int i=0;i<2;i++)
        for(int j=0;j<3;j++)
            affineMat(i,j)=x.at<float>(3*i+j);
    
    return affineMat;
    
}

double CalcAverageLum(const Mat& frame, Quadrangle& qua){
    double lum=0.0;
    int count=0;
    Mat gray;
    if(frame.channels()==3)
        cvtColor(frame, gray, CV_BGR2GRAY);
    for(int i=0;i<gray.rows;i++)
        for(int j=0;j<gray.cols;j++){
            Point2f ptCord = Point2f(j,i);
            if(!IsInsideQuadrangle(qua, ptCord))
                continue;
            lum += frame.at<uchar>(i,j);
            count++;
        }
    
    double averageLum = lum/(double)count;
    return averageLum;
}
