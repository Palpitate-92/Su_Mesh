-- Lua script.
p=tetview:new()
p:load_plc("C:/Users/20758/Desktop/Su_Mesh/Su_Mesh_Two_Dim/Mesh/Before_Quality.smesh")
rnd=glvCreate(0, 0, 500, 500, "TetView")
p:plot(rnd)
glvWait()
