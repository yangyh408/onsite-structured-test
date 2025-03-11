# 运行Docker容器本地环境配置



## Windows

1. 在windows 下**安装wsl**

   + 自动安装方法：https://learn.microsoft.com/en-us/windows/wsl/install
   + 手动安装方法：https://learn.microsoft.com/en-us/windows/wsl/install-manual

   > 若在安装时提示 *无法解析服务器的名称或地址* 可手动修改系统DNS服务器为`114.114.114.114`和`8.8.8.8`，详情参考：https://blog.csdn.net/weixin_43328157/article/details/129052041

2. 安装docker

   + 下载地址：[Docker Desktop for Windows](https://desktop.docker.com/win/main/amd64/Docker Desktop Installer.exe?_gl=1*crnwg9*_ga*MTA5OTE4NDY1LjE3MDIyNjcwMTM.*_ga_XJWPQMJYHQ*MTcxMTA1OTg5MS4xNC4xLjE3MTEwNTk4OTIuNTkuMC4w)

   + 安装方法：https://docs.docker.com/desktop/install/windows-install/

     > 注意：在安装配置选项中推荐勾选"Use WSL 2 instead of Hyper-V (recommended)"

3. 安装XLaunch

   + 下载地址：https://sourceforge.net/projects/vcxsrv/
   + 下载完成后点击exe文件进行安装
   + 安装完成后会在桌面生成`XLaunch`应用图标

4. 运行XLaunch

   > 每次运行docker容器前均需检查XLaunch应用是否保持开启状态
   >
   > 参考文章：https://zhuanlan.zhihu.com/p/128507562

   + 双击`XLaunch`启动应用

   + 配置部分不需要额外设置，一直点击"下一步"直到"完成"

   + 此时在桌面右下角系统托管区看到`XLaunch`的图标即配置完成

     

## Ubuntu 20.04

> 参考文章：https://blog.csdn.net/a806689294/article/details/111462627

1. 安装xserver

   ```bash
   sudo apt install x11-server-utils
   ```

2. 许可所有用户都访问xserver*（每次主机重启后都需要执行一次）* 

   ```bash
   xhost +
   ```

3. 查看当前显示的环境变量

   ```bash
   echo $DISPLAY
   ```

   通常情况下该值为`:0`，若不一致需要修改`docker-compose-ubuntu.yaml`文件中的`display`字段为对应值

   ```yaml
   services:
     TESSNG:
       environment:
         DISPLAY: :0
   ```

   