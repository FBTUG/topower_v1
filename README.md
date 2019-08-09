# topower_v1
## 土砲1號
<img src="https://github.com/aga3134/topower_v1/blob/master/photo/P_20190808_105344.jpg?raw=true" width="480px">

### 目的
本專案目的在提供簡單的範例以ROS架構整合載具驅動、視覺偵測、手臂驅動、搖桿控制，**不包含**3D定位、SLAM、自動控制的部份。
整合項目包括：
- raspberry pi ros 主程式
- rosserial arduino馬達驅動(含雙輪驅動與相機轉動)
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

\* 在raspberry pi上跑gazebo雖然可以開，但可能因為記憶體不足，打開之後會整個當掉。

#### 遠端搖控
在ROS中若要做到兩台機器間互相溝通，兩台**機器需在同一個內網**下(除非你有固定對外的IP)，並且設定**ROS_MASTER_URI和ROS_IP**(或ROS_HOSTNAME，跟ROS_IP二擇一)

設定指令：
- export ROS_MASTER_URI=http://\{執行roscore的機器IP或hostname\}:11311
- export ROS_IP=\{這台機器的IP\} 或
- export ROS_HOSTNAME=\{這台機器的hostname\}
- 若是使用hostname，需在/etc/hosts中寫入ip跟hostname的對應關係

把前面所有東西設定好後，ssh進raspberry pi執行start_demo.sh(裡面會跑topower_v1_demo.launch)，然後在pc中執行topower_v1_remote.launch做遠端操控跟影像辨識。最後打開rqt觀看影像處理結果。

#### 執行結果
搖桿控制包含車體控制跟手臂控制兩種模式，按鈕依羅技F310搖桿設定。如果使用不同搖桿，可進joy_control.launch調整。

##### 車體控制
<img src="https://github.com/aga3134/topower_v1/blob/master/photo/%E8%BB%8A%E9%AB%94%E6%93%8D%E6%8E%A7.jpg?raw=true" width="480px">

##### ros 車體遙控測試
[![ros 車體遙控測試](https://img.youtube.com/vi/kWfkIq2DYRM/0.jpg)](https://www.youtube.com/watch?v=kWfkIq2DYRM)

##### 手臂控制
<img src="https://github.com/aga3134/topower_v1/blob/master/photo/%E6%89%8B%E8%87%82%E6%93%8D%E6%8E%A7.jpg?raw=true" width="480px">

##### ros moveit 手臂測試
[![ros moveit 手臂測試](https://img.youtube.com/vi/ItZ9fYsNWAc/0.jpg)](https://www.youtube.com/watch?v=ItZ9fYsNWAc)

##### ROS接yolov3_tiny假蕃茄測試
在rqt中打開image view plugin，並把topic設成/topower_v1/camera/yolo_result/compressed

[![ROS接yolov3_tiny假蕃茄測試](https://img.youtube.com/vi/KiRTkXGwmpo/0.jpg)](https://www.youtube.com/watch?v=KiRTkXGwmpo)

##### ros接apriltag測試
在rqt中打開image view plugin，並把topic設成/topower_v1/camera/apriltag/compressed

[![ros接apriltag測試](https://img.youtube.com/vi/0ZxaFdWtulE/0.jpg)](https://www.youtube.com/watch?v=0ZxaFdWtulE)

### 資料夾結構
- arduino: 放arduino車體控制程式
	- RosArduinoHelloWorld: arduino跟ros溝通的簡單範例
	- RosArduinoMotorControl: arduino控制車體的程式
- esp8266_arm: 放esp8266手臂控制程式
	- RosESP8266Arm: esp8266手臂控制程式
	- RosESP8266HelloWorld: esp8266跟ros溝通的簡單範例
- topower_v1: ros主程式的package
	- config: 放參數檔
		- cfg.fake-tomato: yolo參數設定，使用yolov3-tiny訓練假蕃茄模型
		- topower_v1_control.yaml: ros control參數設定
		- topower_v1_moveit.rviz: rviz設定for moveit測試
		- topower_v1_rviz.rviz: rviz設定for urdf debug
	- launch: 放launch檔
		- joy_control.launch: 搖桿控制
		- topower_v1_demo.launch: 實體機器控制
		- topower_v1_gazebo.launch: gazebo模擬
		- topower_v1_remote.launch: 遠端控制
		- topower_v1_rviz.launch: 開啟rviz做urdf debug
	- msg:  放message定
		- ArmJoyCmd.msg: 搖桿傳給moveit控制手臂的訊號
		- ArmPose.msg: 傳給esp8266手臂的訊號
		- CamPanTilt.msg: 控制相機轉動的訊號
		- WheelDrive.msg: 控制車子移動的訊號
	- robot_model: 放土砲1號的urdf定義
		- 最上層的定義是topower_v1.xacro，裡面會將其他xacro檔叫進來
		- topower_v1_arm.urdf.xacro、topower_v1_pan_tilt_cam.urdf.xacro、topower_v1_vehicle.urdf.xacro分別定義手臂、相機、車體機構
		- topower_v1_common.xacro定義給其他xacro共用的元素
		- topower_v1.gazebo定義給gazebo看的參數
		- materials.xacro定義在rviz顯示的顏色
	- src: 放主程式
		- apriltag: 偵測apriltag的程式
		- topower_v1: 主要控制程式
			- arm_control_node.py: 手臂控制程式
			- cam_capture_node.py: 相機取像程式
			- car_control_node.py: 車體控制程式
			- joy_mapper_node.py: 搖桿控制程式
		- yolo: 偵測假蕃茄的程式
		- topower_v1_hw.cpp: 定義土炮1號的hardware interface，跟實體機器溝通
	- worlds: 放gazebo場景 
- topower_v1_moveit: 由moveit setup assistant產生的package

### 硬體資料
<img src="https://github.com/aga3134/topower_v1/blob/master/photo/P_20190809_101116.jpg?raw=true" width="480">

- arduino digital out
	- 左輪前進: pin 11
	- 左輪後退: pin 10
	- 右輪前進: pin 6
	- 右輪後退: pin 9
	- 相機左右轉: pin 3
	- 相機上下轉: pin 5
- 所有pin輸出皆需PWM功能，程式使用SoftPWM函式庫，選哪個pin輸出應該都ok
- 相機轉動的伺服馬達直接吃arduino輸出的5v電源
- 雙輪馬達驅動時的突波會影響伺服馬達位置，所以雙輪馬達另外從USB行動電源獨立供電
- 萬向輪固定用兩個螺帽上下夾緊，方便調整車體水平
	- <img src="https://github.com/aga3134/topower_v1/blob/master/photo/P_20190809_103450.jpg?raw=true" width="360">
- [BOM](https://docs.google.com/spreadsheets/d/1zkPduSW5lWat-D1qk9nLzZ4ZvrCBr-DHGEX5C0q9JmQ/edit?usp=sharing)

### 已知問題
- raspberry pi上執行yolov3-tiny跑不起來，可能是記憶體不足，網路上有[範例](http://raspberrypi4u.blogspot.com/2018/10/raspberry-pi-yolo-real-time-object.html)是跑yolov2-tiny，但是很慢。之後需搭配[NCS2](https://www.mouser.tw/new/Intel/intel-neural-compute-stick-2/)或改用[jetson nano](https://www.nvidia.com/zh-tw/autonomous-machines/embedded-systems/jetson-nano/)
- 手臂使用的串列馬達雖然可以設定移動速度，但是使用搖桿控制因為會連續一直傳位置給手臂，速度調快動作會頓、調慢則會lag；如果是直接給一個目標位置一次移動到位則不會有這個問題。
- moveit預設的ik solver對目前使用的手臂容易算不出角度，要實用的話需自己寫轉換或試用其他ik solver
- 車子跑一跑輪子容易掉下來
- apriltag看起來很穩定，但是有部分遮蔽就會抓不到。如果農場植物會長得很雜亂可能不適用
- 截取相機照片如果有開啟上下或左右flip，有時會取到顛倒的畫面
- 夾爪打開或關閉時會抖，有時還會在沒訊號控制的時候自己動

### 特別感謝
- 感謝哈爸借[霹靂車](https://www.icshop.com.tw/product_info.php/products_id/26775)給我參考和超佛心[ros共筆](https://paper.dropbox.com/doc/FBTUG-FarmHarvestBot--AL0ocC8x8bX6TSHoopuJMw0NAg-w2FKkhc4ZTlj6knhOK43p)
- 感謝CH大提供技術咨詢和超佛心[機器學習心得](https://chtseng.wordpress.com/category/%e5%bf%83%e5%be%97-%e6%a9%9f%e5%99%a8%e5%ad%b8%e7%bf%92/)
- 感謝eric提供機械手臂、[手臂文件](https://github.com/ericyangs/5dof_arm)與假蕃茄模型
