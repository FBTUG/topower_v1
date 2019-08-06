# topower_v1
## 土砲1號
### 目的
本專案目的在提供簡單的範例以ROS架構整合載具驅動、視覺偵測、手臂驅動、搖桿控制，不包含3D定位、SLAM、自動控制的部份。
整合項目包括：
- raspberry pi ros 主程式
- rosserial arduino馬達驅動(含雙輪驅動與相機旋轉)
- rosserial tcp esp8266手臂驅動
- urdf建模與gazebo模擬
- ros control 馬達驅動
- movit 手臂路徑規劃
- 搖桿控制
- picamera影像擷取與yolo物件偵測
- apriltag偵測

### 開發環境與程式架構
- PC OS: ubuntu 18.04
- Raspberry PI OS: ubuntu mate 18.04
- ROS version: medolic

主程式放在Raspberry PI，透過rosserial與arduino溝通，由arduino控制車子移動與相機轉動；
同時以rosserial tcp與手臂(esp8266板)溝通，經由moveit輸出移動路徑；
PC端連接搖桿控制車子與手臂，並取得camera影像做假蕃茄和apriltag偵測。

### 安裝
#### PC與Raspberry PI
1. 從github下載程式 git pull https://github.com/aga3134/topower_v1.git 到ros working space的src資料夾
2. 複製libdarknet.so到src/yolo資料夾裡
	- 在src/yolo資料夾裡有libdarknet.so.pc和libdarknet.so.pi兩個預先建置好的lib。在PC上使用就將libdarknet.so.pc改名為libdarknet.so；在Raspberry PI上使用就將libdarknet.so.pi改名為libdarknet.so；如果兩個都不行就從[yolo source code](https://pjreddie.com/darknet/install/)自己編譯。
3. 複製libapriltag.so到src/apriltag資料夾裡
	- 在src/apriltag資料夾裡有libapriltag.so.pc和libapriltag.so.pi兩個預先建置好的lib。在PC上使用就將libapriltag.so.pc改名為libapriltag.so；在Raspberry PI上使用就將libapriltag.so.pi改名為libapriltag.so；如果兩個都不行就從[apriltag source code](https://github.com/AprilRobotics/apriltag)自己編譯。
4. 安裝需要的ros package
	- sudo apt-get install ros-melodic-moveit ros-melodic-rosserial-arduino ros-melodic-rosserial ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-cv-bridge ros-melodic-vision-opencv
5. 在ros working space資料夾執行catkin_make編譯程式

#### arduino

#### 手臂(esp8266)

### 使用

### 資料夾結構

### 硬體資料

### 特別感謝
- 感謝哈爸超佛心[ros共筆](https://paper.dropbox.com/doc/FBTUG-FarmHarvestBot--AL0ocC8x8bX6TSHoopuJMw0NAg-w2FKkhc4ZTlj6knhOK43p)
- 感謝CH大超佛心[機器學習心得](https://chtseng.wordpress.com/category/%e5%bf%83%e5%be%97-%e6%a9%9f%e5%99%a8%e5%ad%b8%e7%bf%92/)
- 感謝eric提供機械手臂、[手臂文件](https://github.com/ericyangs/5dof_arm)與假蕃茄模型
