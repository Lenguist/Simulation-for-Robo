Theres a lotta bullshit that I will organize later (probably not). 

All you need is
1. install pybullet. This was pretty annoying at first but there is info in the slides on how to do it. from what i remember its

*Mac users: If you are running Xcode and have “install collisions”, type:
Xcode-select --install
pip install pybullet
pip install gym

text me if you are having issues installing.

2. run robot_sim.py this starts the simulation, right now running test_robot.urdf and plane.urdf. test_robot.urdf is the main "workspace" i was fiddling with.

Notes:
in line 11, you can change it to robotId = p.loadURDF("**full_robot.urdf**",robotStartPos, robotStartOrientation) and that will run the other urdf. This one uses the full leg instead of just the upper leg.
when u open those urdf files to fiffle around with them, you will notice they depend on an stl file. it is my belief that the stl files contain info about relative positioning. I am using MeshLab to check these. 


I forgot to say one thing when we called, which is that on slide 23 of the Simulation presentation, he explains this setting in Solidworks for exporting STL's, which is that you have to check the box that says "Do not translate STL output data to positive space". 
He said this shifts things to the origin when exporting. We do NOT want this. However, in Fusion, i could not for the life of me find this setting. It may be the source of my pain. Not certain. 

im prolly forgetting other things. oh well. talk to u later

