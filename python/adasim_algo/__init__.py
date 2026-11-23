"""ADASim 算法进程包。

模块划分对应"感知—预测—规划—控制"的经典链路：

    protocol.py   线上协议编解码（与 C++ 网关逐字节镜像）
    perception.py 点云聚类、框拟合、帧间关联、速度估计
    planning.py   CTRV 预测、IDM 跟驰、Stanley 跟踪、执行器滞后
    node.py       事件循环与收发装配

除标准库外无任何依赖，因此算法进程可以在只有 Python 的机器上直接运行，
不需要为 numpy / PyTorch 预留安装步骤。
"""

__version__ = "0.1.0"
