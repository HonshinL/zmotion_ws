


```
g++ src/main.cpp src/zmcaux.cpp \
    -I include \
    -L lib \
    -lzmotion -lpthread \
    -o zmotion_test
```

```
LD_LIBRARY_PATH=./lib ./zmotion_test 192.168.0.11
LD_LIBRARY_PATH=./lib ./get_controller_info 192.168.0.11
```

ros2 run zmotion_driver zmotion_test

ros2 service call /zmc/convert_dxf_to_xml motion_msgs/srv/ConvertDxfToXml "{dxf_file_path: '/home/hongxin/Projects/10_Github/zmotion_ws/src/zmotion_driver/config/example.dxf'}"