
#include "FTrack.hpp"
#include "gms_matcher.h"

FTrack::FTrack(){
    status=1;
    KFcount=0;
    KFfirst=0;
    FailCount=0;

}

int FTrack::Init(const Mat& _frame, const Quadrangle& _qua){
    _frame.copyTo(img_old);

    mInitQua=_qua;
    mIQua=_qua;

    cvtColor(img_old,gray_old,COLOR_BGR2GRAY);

    //Get ORB kps and des
    Mat mask;
    mask = Mat::zeros(img_old.size(),CV_8U);
    Rect r(mIQua.tl,mIQua.br);
    mask(r).setTo(1);

    Size mSz;Rect recForORBs;int ORB_EdgeThres=19;
    mSz = Size(_frame.cols, _frame.rows);
    recForORBs = GetMBRfromQuadr(mIQua,mSz);
    Rect recExpnd = ExpandRectEdge(recForORBs, mSz, ORB_EdgeThres);

    Mat tmp = Mat(gray_old,recExpnd);
    Mat imgForORBs;

    kps1.clear();
    tmp.copyTo(imgForORBs);
    ORBextractor Ini_orber(500,1.2f,1,20,7,false);       //nfeatures,scaleFactor,nlevel,iniThFAST,minTh,buseOrcTree
    Ini_orber(imgForORBs,Mat(),kps1,des1);        //get pre- kps and des

    //get first Qua--center of big Qua
    int CW=cvRound((mIQua.br.x-mIQua.tl.x)/4);
    int CH=cvRound((mIQua.br.y-mIQua.tl.y)/4);
    Point2f CQCenter=GetCenter(mIQua);
    mCQua=GetNewQua(CQCenter,CW,CH);

    points_old.clear();
    points_centerold.clear();
    maskC.clear();

    //Get center points and their mask(maskC)
    for(int i=0;i<kps1.size();i++){
        Point2f tmp = kps1[i].pt+Point2f(recExpnd.x,recExpnd.y);
        if(IsInsideQuadrangle(mCQua,tmp)){
            points_centerold.push_back(tmp);
            maskC.push_back(1);
            maskBC.push_back(1);
        }
        else {
            maskC.push_back(0);
            maskBC.push_back(0);
        }
        points_old.push_back(tmp);

    }

    //need enough center points
    NofCQ=points_centerold.size();
    cout<<"NofCQ: "<<NofCQ<<"maskC: "<<maskC.size()<<endl;
    if(NofCQ<5){

        cerr<<"Center Qua without good features"<<endl;
        return 0;
    }    

    return 1;
}

int FTrack::Process(Mat& _frame){

    if(status==0){
        return 0;
    }
    else if(status==2){
        BF(_frame);
        return 2;
    }

    cv::TermCriteria termcrit_of(2, 30, 0.01);
    maskOF.clear();
    points_new.clear();
    points_centernew.clear();
    nmaskC.clear();

    _frame.copyTo(img_new);
    cvtColor(img_new,gray_new,COLOR_BGR2GRAY);

    double stOF = (double)getTickCount();
    vector<float> err;
    Size winSize(31,31);
    calcOpticalFlowPyrLK(gray_old,gray_new, points_old, points_new, maskOF, err, winSize, 3, termcrit_of, 0, 0.0001);   // OutputArray maskOF(01) ,  err

    double ptOF= ((double)getTickCount()-stOF)/((double)getTickFrequency());

    vector<Point2f> fundP2fs_1 = P2fsToP2fs(points_old, maskOF);
    vector<Point2f> fundP2fs_2 = P2fsToP2fs(points_new, maskOF);

    vector<uchar> fundStates;
    Mat ff = findFundamentalMat(fundP2fs_1,fundP2fs_2,fundStates);

    float fundSurvival_1=( float(CountNonZero(fundStates))) / ( float(fundStates.size()) );
    if(fundSurvival_1<0.2)
        {status=2; return 2;  }      // failed frame
    maskOF = OverlayMask(maskOF,fundStates);

    //optional; find Fundamental twice;maybe only need once
    vector<Point2f> fundP2fs_3 = P2fsToP2fs(fundP2fs_1, fundStates);
    vector<Point2f> fundP2fs_4 = P2fsToP2fs(fundP2fs_2, fundStates);
    fundStates.clear();

    ff = findFundamentalMat(fundP2fs_3,fundP2fs_4,fundStates);
    float fundSurvival_2=( float(CountNonZero(fundStates))) / ( float(fundStates.size()) );
    if(fundSurvival_1*fundSurvival_2<0.2)
        {status=2; return 2;  }    // failed frame
    maskOF = OverlayMask(maskOF,fundStates);

    //final points after OF
    vector<Point2f> fundP2fs_6 = P2fsToP2fs(fundP2fs_4, fundStates);


    //find centers of img1\2
    float sumx=0;float sumy=0;
    for(int i=0;i<fundP2fs_6.size();i++){
        sumx +=fundP2fs_6[i].x;
        sumy +=fundP2fs_6[i].y;
    }
    Point2f center2(sumx/fundP2fs_6.size(),sumy/fundP2fs_6.size());
    vector<pair<Point2f,double>> mvmPn2;  //pair<Point,norm with center>
    for(int i=0;i<fundP2fs_6.size();i++){
        mvmPn2.push_back(make_pair(fundP2fs_6[i],
                                   norm(Point2f((fundP2fs_6[i].x-center2.x),(fundP2fs_6[i].y-center2.y)))));
    }

    //sort points
    vector<pair<Point2f,double>> mvmPn2s(mvmPn2.size());
    copy(mvmPn2.begin(),mvmPn2.end(),mvmPn2s.begin());      //s--sort

    sort(mvmPn2s.begin(),mvmPn2s.end(),
         [](const pair<Point2f,double> &a, const pair<Point2f,double> &b)
                  {
                      return a.second < b.second;
                  });   //升序
    double ratio=0.95;  //delete outlier
    vector<Point2f> fundP2fs_fin2;
    for(int i =0;i<ceil(mvmPn2s.size()*ratio);i++){
        fundP2fs_fin2.push_back(mvmPn2s[i].first);
    }

    Mat show2;
    img_new.copyTo(show2);

    //find and draw contours (optional,only for show)
    vector<int> hull2;
    convexHull(Mat(fundP2fs_fin2), hull2, true);
    Point pt20 = fundP2fs_fin2[hull2[ hull2.size()-1]];
    for(int  i = 0; i < hull2.size(); i++ )
    {
        Point2f pt2 = fundP2fs_fin2[hull2[i]];
        line(show2, pt20, pt2, Scalar(0, 255, 0), 1,LINE_AA);
        pt20 = pt2;
    }

    //boundingRect, get new Rect
    Rect two=boundingRect(fundP2fs_fin2);

    //add points ****have to be tested*****
    int Th=cvRound((mInitQua.br.x-mInitQua.bl.x)/2); //Need to be changed according to the initial Qua  Qua.w/h *0.5
    int ADD=cvRound((mInitQua.br.x-mInitQua.bl.x)/4);
    int MAX_COUNT=100;
    int ThP=250;

    Mat mask1;
    mask1 = Mat::zeros(img_new.size(),CV_8U);

    vector<Point2f> addpoints;

    if(isInPicture(two)){
        cv::TermCriteria termcrit_t(2,30,0.03);
        //To determine if we need to increase the point
        if((two.height<Th || two.width<Th || fundP2fs_fin2.size()<ThP) && fundP2fs_fin2.size()<500){    //Points nember need to be limited

            //try to expand the Rect. if outside picture, don't expand
            Rect tmp(two.tl().x-ADD , two.tl().y-ADD , two.width+2*ADD , two.height+2*ADD);
            if(!isInPicture(tmp)){
                tmp=Rect(two.tl().x-ADD/2 , two.tl().y-ADD/2 , two.width+ADD , two.height+ADD);
            }
            if(isInPicture(tmp)){
                two=tmp;
            }

            //add points inside Rect
            mask1(two).setTo(255);
            goodFeaturesToTrack(gray_new, addpoints, MAX_COUNT, 0.01, 10, mask1, 3, 0, 0.04);
            cornerSubPix(gray_new, addpoints, Size(10,10), Size(-1,-1), termcrit_t);
            for(int i=0;i<addpoints.size();i++){
                points_new.push_back(addpoints[i]);
            }
        }

    }
    else {
        cerr<<"Rect out of Picture!!!"<<endl;
    }

    rectangle( show2, two.tl(), two.br(), Scalar(0, 0, 255), 2, 8, 0 );
    for(int i=0;i<points_new.size();i++){
        circle( show2, points_new[i], 3, Scalar(0,255,0), -1, 8);
    }

    tmpQua=GetQuadrfromRect(two);

    //get new center points
    for(int i=0;i<points_new.size();i++){
        if((maskC[i]==1)&&(maskOF[i]==1)){
            points_centernew.push_back(points_new[i]);
            nmaskC.push_back(1);
        }
        else{
            nmaskC.push_back(0);
        }
    }

    //try to fund homography
    if(points_centernew.size()==points_centerold.size()){

        if(points_centerold.size()>4){
            vector<uchar> homoStates;
            Mat hh =(findHomography(points_centerold,points_centernew,CV_RANSAC,1,homoStates)); //OutputArray mask=homoStatus

            points_centernew = P2fsToP2fs(points_centernew, homoStates);
            cerr<<"homo OK"<<endl;
        }
        else {
            cerr<<"can't homo: "<<points_centernew.size()<<endl;
        }

    }

    //delete points if their norm too big(>4*dis_avg) 4 can be changed
    float dis_max=0.0;
    float dis_avg;float dis_sum=0.0;
    Point2f Center=GetCentroid(points_centernew);
    for(int i=0;i<points_centernew.size();i++){
        float tmpdis = norm(Point2f((points_centernew[i].x-Center.x),(points_centernew[i].y-Center.y)));
        dis_sum += tmpdis;
        if(tmpdis>dis_max){
            dis_max = tmpdis;
        }
    }
    dis_avg = dis_sum/points_centernew.size();
    vector<Point2f>::iterator it;
    for(it=points_centernew.begin();it!=points_centernew.end();){
        if(norm(Point2f(((*it).x-Center.x),((*it).y-Center.y)))>4*dis_avg){
        it=points_centernew.erase(it);
        }
        else
        ++it;
    }

    int CMAX_COUNT=20;  //max number of added center points, should be tested and changed
    Mat maskc;
    maskc = Mat::zeros(img_new.size(),CV_8U);
    Rect Rc=boundingRect(points_centernew);     //Get new small Rect

    tmpCQua=GetQuadrfromRect(Rc);

    if(isInPicture(Rc)){
        //if small Rect too small, we have to expand it
        if( (Rc.width<5||Rc.height<5) && (Rc.tl().x>1) && (Rc.tl().y>1) ) {
            Rect tmpRc(Rc.tl().x-1,Rc.tl().y-1,Rc.width+2,Rc.height+2);
            Rc=tmpRc;
        }
        //if center points too less, add points from big Rect, add points by goodFeature inside small Rect
        if(points_centernew.size()<20){

            //add from big Rect
            cv::TermCriteria termcrit_t(2,30,0.03);
            for(int i=0;i<addpoints.size();i++){
                if(IsInsideQuadrangle(tmpCQua,addpoints[i])){

                    nmaskC.push_back(1);
                    points_centernew.push_back(addpoints[i]);
                }
                else{
                    nmaskC.push_back(0);
                }
            }

            //add from small Rect
            maskc(Rc).setTo(255);
            vector<Point2f> caddpoints;
            goodFeaturesToTrack(gray_new, caddpoints, CMAX_COUNT, 0.01, 10, maskc, 3, 0, 0.04);
            if(caddpoints.size()>0){
                cornerSubPix(gray_new, caddpoints, Size(10,10), Size(-1,-1), termcrit_t);
                for(int i=0;i<caddpoints.size();i++){
                    if( IsInsideQuadrangle(tmpCQua,caddpoints[i]) ){
                        points_centernew.push_back(caddpoints[i]);
                        nmaskC.push_back(1);
                        if(points_new.size()<600){
                            points_new.push_back(caddpoints[i]);
                        }

                    }
                }
            }

        }

    }
    rectangle( show2, Rc.tl(), Rc.br(), Scalar(0, 255, 255), 2, 8, 0 );

    cout<<"points_centernew: "<<points_centernew.size()<<endl;

    if(points_centernew.size()<=1){
        cerr<<"Lack of points "<<endl;
        status=2;
        return 2;
    }

    //imshow("s2",show2);waitKey(1);

    FinalPoint=GetCentroid(points_centernew);

    //updata
    points_old.clear();
    for(int i=0;i<points_new.size();i++){
        points_old.push_back(points_new[i]);
    }
    points_centerold.clear();
    for(int i=0;i<points_centernew.size();i++){
        points_centerold.push_back(points_centernew[i]);
    }
    mIQua=tmpQua;
    mCQua=tmpCQua;
    img_new.copyTo(img_old);
    gray_new.copyTo(gray_old);
    maskC=nmaskC;

    //for KeyFrame
    KFcount++;
    if(KFcount==30){

        float sum_ssd=0.0;
        for(int i=0;i<points_centerold.size();i++){
            sum_ssd += norm(Point2f((points_centerold[i].x-FinalPoint.x),(points_centerold[i].y-FinalPoint.y)));
        }
        if(KFfirst==0){ //only one times
            KFSSD1=sum_ssd;
            KFQua1=mCQua;
            KFfirst++;
        }
        else{
            KFSSD2=sum_ssd;
            KFQua2=mCQua;
            float KFratio =KFSSD2/KFSSD1;
            float CQuaratio = (KFQua2.br.x-KFQua2.bl.x)/(KFQua1.br.x-KFQua1.bl.x);
            cout<<"ratio: "<<KFratio<<"   "<<CQuaratio<<endl;
            if(CQuaratio/KFratio>2.0||CQuaratio/KFratio<0.5){
                //
            }
        }
        KFcount=0;
    }

    status=1;
    return 1;

}

int FTrack::BF(Mat& _frame){
    _frame.copyTo(img_new);
    cvtColor(img_new,gray_new,COLOR_BGR2GRAY);
    vector<KeyPoint> cndKpsF2;
    Mat cndOrbDesF2;

    ORBextractor orber(500,1.2,1,20,7,false);       //nfeatures,scaleFactor,nlevel,iniThFAST,minTh,buseOrcTree
    orber(gray_new,Mat(),cndKpsF2,cndOrbDesF2);        //get pre- kps and des

    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    vector<int> vnMatches12(kps1.size(),-1);
    vector<int> vnMatches21(cndKpsF2.size(),-1);
    vector<float> vMatchedDistance(cndKpsF2.size(),INT_MAX);

    //BF with OrientationTest
    int nmatches=0;
    for(size_t i1=0; i1<des1.rows; i1++)
    {

        Mat d1 = des1.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;    //次近邻
        int bestIdx2 = -1;

        for(int i2=0;i2<cndOrbDesF2.rows;i2++)
        {
            Mat d2 = cndOrbDesF2.row(i2);
            int dist = DistOfOrbFeature(d1,d2);
            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }
        //cout<<bestDist<<endl;
        if(bestDist<=TH_LOW)        //60
        {
            if(bestDist<(float)bestDist2*NNratio) //<次近邻*0.9
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = kps1[i1].angle-cndKpsF2[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }
    cout<<"orinmatchs:  "<<nmatches<<endl;

    //Get new points after BF--just for new Centroid
       for(int i=0;i<nmatches;i++){
            if(vnMatches12[i]>=0){

                if(maskBC[i]==1){
                    points_centernew.push_back(cndKpsF2[vnMatches12[i]].pt);
                    nmaskC.push_back(1);
                }
                else {
                    nmaskC.push_back(0);
                }

            }
        }

       NofCQ=points_centernew.size();
       if(NofCQ<=4){

           cerr<<"Center Qua without good features"<<endl;
           FailCount++;

           //try 50 times; 50 can be changed
           if(FailCount==50){
               status=0;    //close Tracking
           }
           return 2;    //status unchanged, BF again

       }


    //updata


    //new big Rect
    FinalPoint=GetCentroid(points_centernew);
    int CW=cvRound(mInitQua.br.x-mInitQua.tl.x);
    int CH=cvRound(mInitQua.br.y-mInitQua.tl.y);
    mIQua=GetNewQua(FinalPoint,CW,CH);

    //new small Rect
    int CWs=cvRound((mIQua.br.x-mIQua.tl.x)/4);
    int CHs=cvRound((mIQua.br.y-mIQua.tl.y)/4);
    mCQua=GetNewQua(FinalPoint,CWs,CHs);


    //new big points
    Mat maskn = Mat::zeros(img_new.size(),CV_8U);
    Rect big(mIQua.tl,mIQua.br);
    maskn(big).setTo(255);
    cv::TermCriteria termcrit_n(2,30,0.03);
    goodFeaturesToTrack(gray_new, points_old, 500, 0.01, 10, maskn, 3, 0, 0.04);
    cornerSubPix(gray_new, points_old, Size(10,10), Size(-1,-1), termcrit_n);

    //new small points
    for(int i=0;i<points_old.size();i++){

        if(IsInsideQuadrangle(mCQua,points_old[i])){
            points_centerold.push_back(points_old[i]);
            maskC.push_back(1);
        }
        else {
            maskC.push_back(0);
        }

    }
    //update frame
    img_new.copyTo(img_old);
    gray_new.copyTo(gray_old);

    //only for show and test
    Rect small(mCQua.tl,mCQua.br);
    Mat ims;
    _frame.copyTo(ims);
    for(int i=0;i<points_old.size();i++){
        circle( ims, points_new[i], 3, Scalar(0,0,255), -1, 8);
    }
    rectangle( ims, big.tl(), big.br(), Scalar(0, 255, 0), 2, 8, 0 );
    rectangle( ims, small.tl(), small.br(), Scalar(255, 255, 0), 2, 8, 0 );

    drawKeypoints(ims,cndKpsF2,ims);
    imshow("BF",ims);cout<<"psize:  "<<points_old.size()<<endl;

    //back to OF
    cout<<"back to OF"<<endl;
    status=1;
    FailCount=0;

    return 2;

}

Quadrangle FTrack::GetNewQua(Point2f& _center,int _w, int _h){
    int halfw = cvRound(_w/2);
    int halfh = cvRound(_h/2);
    Point2f ntl = Point2f(_center.x-halfw,_center.y-halfh);
    Point2f ntr = Point2f(_center.x+halfw,_center.y-halfh);
    Point2f nbl = Point2f(_center.x-halfw,_center.y+halfh);
    Point2f nbr = Point2f(_center.x+halfw,_center.y+halfh);
    vector<Point2f> np2fs{ntl,ntr,nbl,nbr};
    Quadrangle tmpCQua(np2fs);
    return tmpCQua;
}
bool FTrack::isInPicture(Rect& _rect){
    if(_rect.tl().x<0||_rect.tl().y<0||_rect.br().x>PW||_rect.br().y>PH||_rect.width>PW||_rect.height>PH){
       return false;
    }
    else
        return true;

}
void FTrack::GetQua(Quadrangle& _qua){
    _qua = mIQua;
    return;
}
Point2f FTrack::GetCenter(Quadrangle& _qua){
    Point2f tmp((_qua.tl.x+_qua.br.x)/2,(_qua.tl.y+_qua.br.y)/2);
    return tmp;
}
void FTrack::GetFinalPoint(Point2f& _FP){
    _FP=FinalPoint;
    return;
}
Point2f FTrack::GetCentroid(vector<Point2f>& _Points){

    float sum_x=0.0;float sum_y=0.0;
    for(int i=0;i<_Points.size();i++){
        sum_x += _Points[i].x;
        sum_y += _Points[i].y;
    }
    Point2f Center(sum_x/_Points.size(),sum_y/_Points.size());
    return Center;
}
int FTrack::GetStatus(){
    return status;
}
