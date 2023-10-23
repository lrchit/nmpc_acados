
这是在python版生成模型与求解器后，使用C++调用C代码库，传入数据进行求解的例程。

C++版速度比 python 快，在Cheetah—Software内实测，horizon=26 时，0.01s内基本能够算完，若减小 horizon，100Hz是没问题的。

改参数的时候需要把 SRBD_Optimizer 函数最后一个参数 debug_state 输入1（其实这个参数也没啥用，
只是方便写python的时候可以写个小demo测试模型），这样会重新由模型创建新的ocp，并写成文件，要加载一会儿。


生成 c code后在build文件夹里 :
cmake ..
make -j

运行：
./nmpc_caller

模型用的四足机器人，改参数和模型见python代码。

