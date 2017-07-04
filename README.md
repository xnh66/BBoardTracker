声明类
```
    FTrack ft;
```

初始化：
@输入：image-图像，initQua-初始框（矩形框由四个点得到）
@输出：初始化结果：1-初始化成功，0-中心框特征点过少

```
vector<Point2f> p2fs{tl,tr,bl,br};
Quadrangle initQua(p2fs);

ft.Init(image, initQua);
```
追踪：
@输入：image ---图像
@输出：追踪状态：1-跟踪稳定，2-跟踪不稳定（调用暴匹），3-跟踪失败

```
TrackResult=ft.Process(image);
```

数据接口：
```
ft.GetQua(qua);	//得到大框
ft.GetFinalPoint(Center);	//得到最终质心点
ft.GetStatus();				//得到跟踪状态
```

