# Slam_Dataloader
Some simple dataloader for slam.

# Components
- [x] General data loader.
    - [x] Euroc format data loader.
    - [x] Custom format data loader.
- [x] Mixed data loader.
    - [x] Multi-sensor synchronized data loader.

# Dependence

### Project repositories
- Slam_Utility

### Third-party repositories
- Eigen3 (>= 3.3.7)（`sudo apt install libeigen3-dev`）
- dw (`sudo apt install libdw-dev`)

# Compile and Run
- 第三方仓库的话需要自行 apt-get install 安装
- 拉取 Dependence 中的源码，在当前 repo 中创建 build 文件夹，执行标准 cmake 过程即可
```bash
mkdir build
cmake ..
make -j
```
- 编译成功的可执行文件就在 build 中，具体有哪些可执行文件可参考 run.sh 中的列举。可以直接运行 run.sh 来依次执行所有可执行文件

```bash
sh run.sh
```

# Tips
- 欢迎一起交流学习，不同意商用；
