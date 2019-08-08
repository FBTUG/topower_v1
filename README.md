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
2. 將yolo的函式庫libdarknet.so複製到src/yolo資料夾裡
	- 如果在PC上使用可將src/yolo資料夾裡的libdarknet.so.pc改名為libdarknet.so
	- 如果在Raspberry PI上使用可將src/yolo資料夾裡的libdarknet.so.pi改名為libdarknet.so；
	- 如果兩個都不能用就從[yolo source code](https://pjreddie.com/darknet/install/)自己編譯。
3. 將apriltag函式庫libapriltag.so複製到src/apriltag資料夾裡
	- 如果在PC上使用可將src/apriltag資料夾裡的libapriltag.so.pc改名為libapriltag.so；
	- 如果在Raspberry PI上使用可將src/apriltag資料夾裡的libapriltag.so.pi改名為libapriltag.so；
	- 如果兩個都不行就從[apriltag source code](https://github.com/AprilRobotics/apriltag)自己編譯。
4. 安裝需要的ros package
	- sudo apt-get install ros-melodic-moveit ros-melodic-rosserial-arduino ros-melodic-rosserial ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-cv-bridge ros-melodic-vision-opencv
5. 在ros working space資料夾執行catkin_make編譯程式
6. 在ros working space資料夾執行source devel/setup.bash更新ros狀態

#### arduino
1. 在arduino放函式庫的資料夾執行 rosrun rosserial_arduino make_libraries.py . 產生ros自定義的message
2. 將arduino/RosArduinoMotorControl/RosArduinoMotorControl.ino透過arduino IDE燒進arduino uno板。

#### 手臂(esp8266)
1. 在arduino放函式庫的資料夾執行 rosrun rosserial_arduino make_libraries.py . 產生ros自定義的message
2. 手臂控制板esp8266可用arduino IDE燒入程式，但需先安裝相關檔案。安裝方式可參考[這裡](http://yhhuang1966.blogspot.com/2017/09/arduino-ide-esp8266.html)
3. 修改esp8266_arm/RosESP8266Arm/RosESP8266Arm.ino裡的ssid、password，並把server的IP address設為執行roscore的機器IP
4. 將esp8266_arm/RosESP8266Arm/RosESP8266Arm.ino透過arduino IDE燒進esp8266板。

### 使用
#### 可透過topower_v1/launch資料夾中的launch檔執行程式
- joy_control.launch 搖桿控制功能
- topower_v1_demo.launch 實體機器控制主程式，包含手臂、車子、相機之間的控制與溝通
- topower_v1_gazebo.launch 開啟gazebo做土砲1號模擬控制
- topower_v1_remote.launch pc端遠端操控實體機器，包含搖桿控制、yolo物件偵測、apriltag偵測
- topower_v1_rviz.launch 開啟rviz將目前狀態做視覺化顯示

\* 如果是用ssh連入機器，因為沒有視窗界面，gazebo跟rviz都無法開啟。

\* 在raspberry pi上跑gazebo雖然可以開，但可能因為記憶體不足打開之後會整個當掉。

#### 遠端搖控
在ROS中若要做到兩台機器間互相溝通，兩台機器需在同一個內網下(除非你有固定對外的IP)，並且設定ROS_MASTER_URI和ROS_IP(或ROS_HOSTNAME，跟ROS_IP二擇一)

設定指令：
- export ROS_MASTER_URI=http://\{執行roscore的機器IP或hostname\}:11311
- export ROS_IP=\{這台機器的IP\} 或
- export ROS_HOSTNAME=\{這台機器的hostname\}
- 若是使用hostname，需在/etc/hosts中寫入ip跟hostname的對應關係

把前面所有東西設定好後，ssh進raspberry pi執行start_demo.sh(裡面會跑topower_v1_demo.launch)，然後在pc中執行topower_v1_remote.launch做遠端操控跟影像辨識。最後打開rqt觀看影像處理結果。

#### 執行結果
搖桿控制包含車體控制跟手臂控制兩種模式，按鈕依羅技F310搖桿設定。如果使用不同搖桿，可進joy_control.launch調整。
##### 車體控制
<img src="https://github.com/aga3134/topower_v1/blob/master/%E8%BB%8A%E9%AB%94%E6%93%8D%E6%8E%A7.jpg?raw=true" width="480px">

##### ros 車體遙控測試
[![ros 車體遙控測試](https://img.youtube.com/vi/kWfkIq2DYRM/0.jpg)](https://www.youtube.com/watch?v=kWfkIq2DYRM)

##### 手臂控制
<img src="https://github.com/aga3134/topower_v1/blob/master/%E6%89%8B%E8%87%82%E6%93%8D%E6%8E%A7.jpg?raw=true" width="480px">

##### ros moveit 手臂測試
[![ros moveit 手臂測試](https://img.youtube.com/vi/ItZ9fYsNWAc/0.jpg)](https://www.youtube.com/watch?v=ItZ9fYsNWAc)

##### ROS接yolov3_tiny假蕃茄測試
在rqt中打開image view plugin，並把topic設成/topower_v1/camera/yolo_result/compressed
[![ROS接yolov3_tiny假蕃茄測試](https://img.youtube.com/vi/KiRTkXGwmpo/0.jpg)](https://www.youtube.com/watch?v=KiRTkXGwmpo)

##### ros接apriltag測試
在rqt中打開image view plugin，並把topic設成/topower_v1/camera/apriltag/compressed
[![ros接apriltag測試](https://img.youtube.com/vi/0ZxaFdWtulE/0.jpg)](https://www.youtube.com/watch?v=0ZxaFdWtulE)

### 資料夾結構

### 硬體資料

### 特別感謝
- 感謝哈爸超佛心[ros共筆](https://paper.dropbox.com/doc/FBTUG-FarmHarvestBot--AL0ocC8x8bX6TSHoopuJMw0NAg-w2FKkhc4ZTlj6knhOK43p)
- 感謝CH大超佛心[機器學習心得](https://chtseng.wordpress.com/category/%e5%bf%83%e5%be%97-%e6%a9%9f%e5%99%a8%e5%ad%b8%e7%bf%92/)
- 感謝eric提供機械手臂、[手臂文件](https://github.com/ericyangs/5dof_arm)與假蕃茄模型
