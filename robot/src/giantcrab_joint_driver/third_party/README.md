
请把创芯 / ZLG 兼容 SDK 的以下文件自行放到系统可见位置：

1. 头文件
   - controlcanfd.h
   - config.h

2. 共享库
   - libcontrolcanfd.so

推荐两种方式：

方式 A（最简单）
- 把 `libcontrolcanfd.so` 复制到本包安装后的 `install/giantcrab_joint_driver/lib/` 下。
- 然后直接运行节点。

方式 B（更灵活）
- 导出环境变量：
  export CONTROLCANFD_SO=/你的实际路径/libcontrolcanfd.so

头文件建议：
- 放到系统 include 路径，或者修改 CMakeLists.txt 增加 include_directories。
- 本包源码默认直接 `#include "controlcanfd.h"` 和 `#include "config.h"`。
