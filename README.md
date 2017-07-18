
声明
```
    FTrack ft;
```


初始化：
@输入：image-图像，initQua-初始框（矩形框由四个点得到）
@输出：初始化结果：true-初始化成功，false-初始化失败，没必要开始tracking

```
vector<Point2f> p2fs{tl,tr,bl,br};
Quadrangle initQua(p2fs);

bool initResult = ft.Init(image, initQua);
```

追踪：
@输入：image ---图像
@输出：追踪状态：0-跟踪失败，结束tracking ;  1-跟踪稳定; 2-跟踪不稳定（开始调用暴匹）

```
int trackResult=ft.Process(image);
```

数据接口：
```
ft.GetQua(qua);	//得到大框,Image坐标系下
ft.GetFinalPoint(Center);	//得到最终靶点
```


备注：
```
image的size由调用方决定，并且保证Init和Process过程的image size永远保持一致。
最终靶点的坐标是在image size决定的坐标系之下的。

src/FTrack.hpp:5 
5: //#define SKIP_BF
将这句话注释掉之后，即可以去掉暴匹过程。

```
