# AB3DMOT 的 Rust 实现

_吐槽一下，Markdown 的呈现方式真是多变，每个平台都有自己独特的方式，导致这一切都不统一_

[AB3DMOT 原项目地址](https://github.com/xinshuoweng/AB3DMOT)  
这个仓库里的代码基本上按照上述仓库中的代码改写而来的~

说一下文件结构：

## 文件结构

```=plain text
thundermot_with_rust
  |
  |-- data/  (数据集)
  |     |
  |     |-- KITTI/  (KITTI 中检测数据集，检测结果(2d + 3d))
  |           |
  |           |-- pointrcnn_Car_test/  (pointrcnn 中 Car 分类的检测框)
  |           |     |
  |           |     |-- 0000.txt  (一段连续的帧文件，其中每一行的格式会在之后说明)
  |           |     |-- ...
  |           |
  |           |-- pointrcnn_Car_val/  (内部格式同上)
  |           |-- pointrcnn_Cyclist_test/  (内部格式同上)
  |           |-- pointrcnn_Cyclist_val/  (内部格式同上)
  |           |-- pointrcnn_Pedestrian_test/  ((内部格式同上)
  |           |-- pointrcnn_Pedestrian_val/  (内部格式同上)
  |           |-- resources/  (内部格式暂不重要)
  |
  |-- src/  (库源文件 
  |     |
  |     |-- lib.rs  (定义了 iou3d(计算两个3d检测框之间的 3d iou 值的函数)、associate_detections_to_trackers(将检测结果和tracker关联的函数)、AB3DMOT(整个模型的实现类)
  |     |-- kalman_filter.rs  (定义了用于合理计算这个问题中bbox下一步位置的 kalman filter)
  |     |-- ab3dmot/  (与 ab3dmot 模型相关，无用，暂时保留)
  |     |     |
  |     |     |-- mod.rs  (定义了 iou3d(计算两个3d检测框之间的 3d iou 值的函数)、associate_detections_to_trackers(将检测结果和tracker关联的函数)、AB3DMOT(整个模型的实现类)
  |     |     |-- kalman_filter.rs  (定义了用于合理计算这个问题中bbox下一步位置的 kalman filter)
  |     |
  |     |-- data/  (与数据输入输出相关)
  |           |
  |           |-- mod.rs  (主要是载入之后两个文件)
  |           |-- input.rs  (输入相关)
  |           |-- output.rs  (输出相关)
  |     
  |-- tests  (测试文件夹)
  |     |
  |     |-- lib.rs  (测试文件的顶层模组，其中定义下方两个模组)
  |     |-- ab3dmot.rs  (测试整体模型)
  |     |-- data.rs  (测试数据输入输出)
  |
  |-- Cargo.toml  (整个项目的 Cargo 配置文件)
```

## 代码结构

```plain text

                                                       AB3DMOT
                                                         |
                                                         |
                                                        \./
data_set.txt -> data::input::prepare_data() -> Frame > update() ------------------>>>> data::output::Objects
                                                       |  |                       ^  |
                                                       |  associate --------------|  |
                                                       |  |  |                 ^  |  |
                                                       |  |  kalman.update ----|  |  |
                                                       |  |  |                 |  |  |
                                                       |  |  |                 |  |  |
                                                       |  |  |-----------------|  |  |
                                                       |__|_______________________|__|

```
