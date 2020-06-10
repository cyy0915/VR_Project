# VR_Project
Final project for CS-238 @ SJTU

# 当前进度
目前source.cpp是点云，meshDivide.cpp是网格，目前两者不相容，编译时只能只能保留一个，另一个从visual studio中移除。

注意，要使纹理正常显示，要将两个obj文件夹中的png或jpg复制到Media/materials/textures中

# TODO

kd树代码中存在内存泄漏的问题，部分内存泄漏的代码位置已注释，未更改

代码结构有待优化，比如将两个cpp中的kd树代码放到一个头文件中，共用

程序完成度可以更高，比如自由选择点云/网格渲染，自由选择两个模型
