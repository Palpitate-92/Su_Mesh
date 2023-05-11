-- Lua script.
p=tetview:new()
p:load_mesh("C:/Users/20758/Desktop/Su_Mesh/Su_Mesh_VS/MESH/feiji/Final_Delaunay.ele")
rnd=glvCreate(0, 0, 500, 500, "TetView")
p:plot(rnd)
glvWait()
