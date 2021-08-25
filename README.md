環境構築は以下のコマンドを実行

sudo apt install libv4l-dev v4l-utils qv4l2

git clone https://github.com/ros-perception/image_common.git

catkin build

git clone https://github.com/ros-drivers/camera_umd.git

catkin build

git clone -b melodic https://github.com/ros-perception/vision_opencv.git

catkin build

git clone https://github.com/ros-perception/image_pipeline.git

catkin build
