
#include "AR_utils.hpp"
#include "FTrack.hpp"


int main(int argc, char **argv)
{

    VideoCapture f;
    f.open(0);
    if (!f.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    int nImages = 800;
    int ccount=0;

    Point2f tl = Point2f(220,140);
    Point2f tr = Point2f(420,140);
    Point2f bl = Point2f(220,340);
    Point2f br = Point2f(420,340);
    vector<Point2f> p2fs{tl,tr,bl,br};
    Quadrangle initQua(p2fs);

    double fps=0;

    cv::Mat image;
    Size si(640,480);

    FTrack ft;      //declare

    while(cvWaitKey(30)<0){
        f>>image;
        if(image.empty())
            break;

        line(image, initQua.tl, initQua.tr, cv::Scalar(255,0,0,255));
        line(image, initQua.tr, initQua.br, cv::Scalar(255,0,0,255));
        line(image, initQua.br, initQua.bl, cv::Scalar(255,0,0,255));
        line(image, initQua.bl, initQua.tl, cv::Scalar(255,0,0,255));

      imshow("live",image);

    }
    for(;;)
    {
        // Read and resize image from file
         f>>image;
         if(image.empty())
             break;
         resize(image,image,si);

        if(ccount==0){

           if(ft.Init(image, initQua)){     //initialization
               cout<<"Init OK"<<endl;
               ccount++;
               continue;
           }
           else {
               cerr<<"Init failed"<<endl;
                continue;
           }
         }

        double st = (double)getTickCount();
        int TrackResult;


        TrackResult=ft.Process(image);
        cout<<"TrackResult: "<<TrackResult<<endl;   //1-stable;2-unstable;3-fail


        if(TrackResult==3){
            cerr<<"over!"<<endl;
            break;
        }


        double pt= ((double)getTickCount()-st)/((double)getTickFrequency());
        fps+=pt;
        cout<<" Process time "<<pt<<endl;


        Quadrangle qua;
        ft.GetQua(qua);
        Point2f Center;
        ft.GetFinalPoint(Center);
        cout<<"final position:  "<<Center<<endl;

        line(image, qua.tl, qua.tr, cv::Scalar(255,0,0,255));
        line(image, qua.tr, qua.br, cv::Scalar(255,0,0,255));
        line(image, qua.br, qua.bl, cv::Scalar(255,0,0,255));
        line(image, qua.bl, qua.tl, cv::Scalar(255,0,0,255));
        circle( image, Center, 10, Scalar(0,0,255), -1, 8);
        imshow("img",image);waitKey(1);


    }
    fps=nImages/fps;
    cout<<"fps:  "<<fps<<endl;

    return 0;
}
