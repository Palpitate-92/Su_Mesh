-- Lua script.
p=tetview:new()
p:load_mesh("E:/Su_Mesh/Su_Mesh_VS/MESH/1111/Removal_1_Delaunay.ele")
rnd=glvCreate(0, 0, 500, 500, "TetView")
p:plot(rnd)
glvWait()
