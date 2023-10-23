# ACADOS_Example

安装 acados 时要把 acados/CMakeist.txt 里面 option(ACADOS_PYTHON "Python Interface" OFF)
改为 option(ACADOS_PYTHON "Python Interface" ON)

遇到其他奇葩问题，解决方法见https://zhuanlan.zhihu.com/p/635857439

在 make examples_c 如果报 config->opts_set 的错，直接找到文件把那一行注释掉。（不知道不做 make_examples_c 和 make run_examples_c 行不行）
