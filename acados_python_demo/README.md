
这是一个用来做四足机器人非线性mpc的示例，用python调CasADi生成动力学模型并创建ocp问题，
acados会生成C代码，可以用直接写python调用，但速度不够C++快，acados_cpp_demo 文件夹里是C++调用的例子。

使用的HPIPM求解器。

改参数的时候需要把 SRBD_Optimizer 函数最后一个参数 debug_state 输入1，
这样会重新由模型创建新的ocp，并写成文件。

我写的python代码比较乱，对acados的使用也不是很规范，而且我没有写参数文件，直接在 srbd_ocp_setting.py 内改参数，那几个.json文件是acados库自动生成的，
运行的时候会调用。缺点是改参数后再运行C++版需要重新编译，好不烦人。


运行: python3 main.py 生成 c code 

