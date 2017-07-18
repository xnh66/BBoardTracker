//
//  AR_utils.hpp
//  SY_AR
//
//  Created by Xnh on 2017/1/5.
//  Copyright © 2017年 Xnh. All rights reserved.
//

#ifndef AR_utils_hpp
#define AR_utils_hpp

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

static float PI=3.1415926;

struct Quadrangle{
    Point2f tl;
    Point2f tr;
    Point2f bl;
    Point2f br;
    Quadrangle(){}
    Quadrangle(vector<Point2f>& p2fs){
        tl=p2fs[0];
        tr=p2fs[1];
        bl=p2fs[2];
        br=p2fs[3];
    }
};

Quadrangle GetQuadrfromRect(const cv::Rect& rec);
cv::Rect GetMBRfromQuadr(const Quadrangle& qua, const cv::Size& imgSize);

vector<Point2f> KpsToP2fs(const vector<KeyPoint>& kps, const vector<uchar> mask);

vector<Point2f> P2fsToP2fs(const vector<Point2f>& inP2fs, const vector<uchar> mask);

Quadrangle GetNewQua(const Quadrangle& inQua, Matx33f& H);

vector<uchar> OverlayMask(const vector<uchar>& mask_1, const vector<uchar>& mask_2);

bool CheckOverlay(const vector<uchar>& originMask, const vector<uchar>& overlayedMask);

vector<Point2f> RemoveKMatrix(const vector<Point2f>& inP2fs, const Matx33f& K);


void GetCentrAndSqrR(const vector<Point2f>& p2fs, Point2f& centroid, float& sqrRadius);

Point2f GetCentr(const vector<Point2f>& p2fs, const vector<uchar> mask);

bool IsInsideQuadrangle(const Quadrangle& qua, const Point2f& testPt);

Point3f CalcLineByTwoP2fs(const Point2f& startP2f, const Point2f& endP2f);

float Det(Matx33f inM);

float CalcQuaArea(const Quadrangle& qua);

float CalcPtDist(const Point2f& pt1, const Point2f&pt2);

void RMatrixToEulerRadians_1X2Y3Z(const Matx33f& R, Vec3f& eulerRadians);

Matx33f EulerToRx(float xRadians);
Matx33f EulerToRy(float yRadians);
Matx33f EulerToRz(float zRadians);

cv::Rect ExpandRectEdge(const cv::Rect& recIn, const cv::Size& sz, const int edge);

int CountNonZero(const vector<uchar>& stats);

float DistOfP2f(const Point2f& p2f_1, const Point2f& p2f_2);

int DistOfOrbFeature(const Mat& feature_1, const Mat& feature_2);

float CalcMatCond(const Matx33f& Hmat);

int ratioTest(vector<vector<cv::DMatch> > &matches, float _ratio);  // for check the orb match result

void symmetryTest( const std::vector<std::vector<cv::DMatch> >& matches1,
                  const std::vector<std::vector<cv::DMatch> >& matches2,
                  std::vector<cv::DMatch>& symMatches );


vector<DMatch> OrbMatch(const Mat& trainDes, const Mat& queryDes, float threshold=64);

vector<vector<DMatch> > OrbMatch_2(const Mat& trainDes, const Mat& queryDes, float threshold=64);

Matx23f AffineMatOfQuas(const Quadrangle& qua_1, const Quadrangle& qua_2);

double CalcAverageLum(const Mat& frame, Quadrangle& qua);

bool CheckRect(const Rect& _rect, int _W, int _H, int _minLength);

Rect ModifyRect(const Rect& recIn, int _W, int _H);
#endif /* AR_utils_hpp */
