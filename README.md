# 4T-POM-X4/4T四足机器人开发套件

4T四足机器人开发套件形态预览图

型号：4T-POM-X4

<div style="text-align: center;">   <img src="%5B7%5D图片/四足机器人.png" alt="四足机器人
 " width="40%" style="display: inline-block;">   </div> 



## 四足机器人开发套件介绍

4T 四足机器人开发套件基于 POM_Nano_G4、POM_Nano_C6 、POM_Nano_RP三款核心控制模组（兼容三种型号），搭配 32 路舵机控制板（POM_SC32）开发而成，具备 12 自由度高性能运动能力。机身采用高强度轻量化 PETG 结构，搭载 12 个高速高性能空心杯舵机，配合连杆结构设计的腿部，动作灵活丰富；内置逆运动学解析、站立姿态控制、步态控制等算法，可轻松实现姿态调节及慢走、小跑等运动模式，支持前进 / 后退、转向、横向移动等功能。

支持上位机和手机蓝牙app快速上手。



## 核心器件

- POM核心模组

  

  <div style="text-align: center;">   <img src="%5B7%5D图片/POM_Nano核心控制模组.png" alt="POM_Nano核心控制模组" width="30%" style="display: inline-block;">   </div> 

  点击查看商品详情

  ​	POM_Nano_G4核心控制模组：https://item.taobao.com/item.htm?id=974067354427&spm=a1z10.5-c-s.w4002-25798805599.14.4e0f309cQg0gvx

  ​	POM_Nano_C6核心控制模组：https://item.taobao.com/item.htm?id=973535587982&spm=a1z10.5-c-s.w4002-25798805599.12.4e0f309cQg0gvx

  ​	POM_Nano_RP核心控制模组：https://item.taobao.com/item.htm?id=974775109430&spm=a1z10.5-c-s.w4002-25798805599.16.4e0f309cQg0gvx

  相关资料获取地址：

  ​	POM_Nano_G4核心控制模组：

  ​							Github 仓库地址：https://github.com/4T-tech/POM_Nano_G4 

  ​							Gitee   仓库地址：https://gitee.com/fourT-tech/POM_Nano_G4

  ​	POM_Nano_C6核心控制模组：

  ​							Github 仓库地址：https://github.com/4T-tech/POM_Nano_C6 

  ​							Gitee   仓库地址：https://gitee.com/fourT-tech/POM_Nano_C6

  ​	POM_Nano_RP核心控制模组：

  ​							Github 仓库地址：https://github.com/4T-tech/POM_Nano_RP 

  ​							Gitee   仓库地址：https://gitee.com/fourT-tech/POM_Nano_RP

- 32路舵机驱动板

  32 路舵机控制器由北京四梯科技有限公司四梯教研团队专为高精度舵机运动控制场景设计开发。该产品集成多路舵机驱动、通信接口和电源管理模块，适用于机器人、自动化设备、教学实验及创意项目等领域。通过板载 TTL 串口，用户可便捷实现与主控设备的通信及二次开发，并支持外接 3S/4S/5S 锂电池供电，满足不同动力场景需求。产品配套 PC 端图形化编程软件，提供从参数调试到程序烧录的全流程支持，板载 4M FLASH 与信号隔离设计，兼顾系统稳定性与脱机运行能力。
  
  提供**上位机**，快速开发。

<div style="text-align: center;">   <img src="%5B7%5D图片/32路舵机控制板.png" alt="32路舵机控制板" width="30%" style="display: inline-block;">   </div> 

​	点击查看商品详情：https://item.taobao.com/item.htm?id=973526331858&spm=a1z10.5-c-s.w4002-25798805599.10.4e0f309cQg0gvx	

​	相关资料获取地址：

​					Github 仓库地址：https://github.com/4T-tech/POM_SC32

​					Gitee   仓库地址：https://gitee.com/fourT-tech/POM_SC32



## 文件结构

- [1]控制算法与固件：4T 四足机器人开发套件固件源码工程
  - Dog_Contorl_SDK_G4：基于POM_Nano_G4核心控制模组开发的机器人控制固件工程文件
  - Dog_Contorl_SDK_C6：基于POM_Nano_C6核心控制模组开发的机器人控制固件工程文件
  - Dog_Contorl_SDK_RP：基于POM_Nano_RP核心控制模组开发的机器人控制固件工程文件
  - 四足机器人固件库使用指南：逆运动学解析、站立姿态控制、步态控制（慢走 / 小跑）等核心算法的说明；
  
- [2]机器人模型：单片机设计与开发实训指导书配套例程
  - 3D打印文件：3mf格式打印件，可直接打印
  - 各部位源文件：stp格式文件，包含机器人大腿、小腿、身体等部件
  - 总装配文件：stp格式文件，机器人所有部件装配关系
  - 模型预览图
  
- [3]硬件设计PCB

  机器人背部通用扩展板和xt30电池接口板硬件设计PCB相关文件。

  - BOM
  - Gerber
  - image：机器人背部通用扩展板和xt30电池接口板预览图
  - PCB：工程文件

- [4]蓝牙app
  
  - 4T-POM-X4_Ctrl_app.apk：Android 的 APK 文件，可直接安装使用
  - 4T-POM-X4_Ctrl_demo.aia：App Inventor工程文件
  - 四足机器人蓝牙app使用及开发说明：app 连接机器人的步骤、控制按钮的功能和二创指南

- [5]材料清单
  
  - 3D打印件清单.xlsx：各打印件的名称、数量、推荐材料（PETG）
  - 元器件清单.xlsx：12 个空心杯舵机、POM_Nano 控制模组、POM_SC32 PCB 板等的规格、数量
  - 五金件清单.xlsx：螺丝、螺母等的规格数量
  
- [6]快速上手指南
  
- [7]图片：附图



## 订购渠道

① 官方淘宝：[gxct.taobao.com]()

② 四梯商城：https://www.4t.wiki/mall

③ 官方京东：https://mall.jd.com/index-16359606.html



## 学习资源获取

- QQ交流群：912962535

- 四梯评测网获取更多资讯：https://4t.wiki/
  - 交流社区：https://www.4t.wiki/community
  - 学习资源：https://www.4t.wiki/curriculum

- Bilibili平台：更多机器人新玩法，B站搜索“四梯科技”

- 微信公众号：微信搜索“四梯”

<div style="text-align: center;">   <img src="%5B7%5D图片/4T_B站.png" alt="4T_B站" width="20%" style="display: inline-block;">   <img src="%5B7%5D图片/4T_公众号.png" alt="4T_公众号" width="20%" style="display: inline-block;"> </div> 
