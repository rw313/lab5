## Lab 5
**David Lee (jl4397)**
**Rachel Wu (rww2115)**  

*COMSW4733 Computational Aspects of Robotics*  
*Peter Allen*

#### To Run
Unzip tarball and `cd` into this repository
```
$ cd <path_to_this_repo>
```

In one terminal run the world file. To launch turtlebot and map for part 1
```
roslaunch followbot launch.launch
```

To launch turtlebot and map for part 2
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
```

To launch turtlebot and map for part 3
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
```

To launch the map for extra credit
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=extra.world
```

In another terminal, run the script in the `src` directory.

For part 1: `python p1.py`  
For part 2: `python p2.py`  
For part 3:  python p3.py
#### Files and Methods
**Part 1**
p1.py
- `Follower`: class that provides logic for the bot's movements
- `image_callback()`: Callback passed into the image subscriber. Creates a binary mask based on the robot's camera's input and follows the line that the mask forms.

**Part 2**
p2.py 
- `Follower`: class that provides logic for the bot's movements. Initialized with several values corresponding to the lower and upper HSV bounds for the colors on the map
- `get_mask_for_color()`: given an HSV frame and a color, this function returns 20 pixels of what the robot sees masked with the color. If, for example, 'red' is passed in as an argument, the resulting image would show white for a pixel if that pixel was red in the original image.
- `rotate()`: rotates the bot
- `image_callback()`: the image callback function passed into the imag subscriber. For every frame, it checks if the centroid of the mass of colored pixels is a red, blue, green, or yellow pixel in the original image. It turns left if the pixel is green, right if the pixel is blue, stops if the pixel is red, and follows the line if the pixel is yellow.

**Part 3**
p3.py
- `Follower` : class that provides logic for the bot's movements. Initialized with several values corresponding to the lower and upper HSV bounds for the yellow and red color
- `get_mask_for_color()` : given an HSV frame and color, this function returns 20 pixels of what the robot sees masked with the color. It works the same as in part 2.
- `rotate()`: rotates the bot left or right. 
- `image_callback()`: same concept as part 2, except this time it uses opencv to detect contours. Then using properties of the contours, it determines what direction the triangle is pointing. In our case, we use the centroid of the triangle and find the furthest point from it in the contour. That is the narrowest angle in the triangle and therefore the direction. If it falls on the left side of thecentroid of the yellow path, then we should turn left accordingly. 

#### Video
Link: https://youtu.be/LCfubL0BRt8 
