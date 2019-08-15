看这里代码的时候先看你们要用的launch文件是什么,然后根据launch文件来找自己需要看的代码

launch:
simulator.launch 执行仿真.纯粹的仿真,传感器数据来自模拟,不进行网络传输
simulator_tcp.launch 执行仿真,传感器数据来自模拟,进行网络传输,由于网络传输只是多开一个节点而已,所以一般情况下不开
repetition.launch 使用socket传输信息并画图重现轨迹
CMC.launch 与CMC进行串口通信,CMC在环




