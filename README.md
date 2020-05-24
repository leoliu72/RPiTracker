# RPiTracker
<p float="left">
  <img src="/images/demo.gif" width="270" />
  <img src="/images/setup.jpg" width="270" height="480" /> 
</p>

## File Structure
* `detection.py`: Detection class used to find the center of the ball.
* `pid.py`: PID class used to calculate the appropriate servo angle.
* `tracking.py`: Brings everything together in a nice loop.
## Running the code
1. Set up OpenCV in your work environment. I have mine in a virtual environment, following [this tutorial](https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/) from Adrian Rosebrock of PyImageSearch.
1. `python tracking.py`
## Motivation
The idea for this project came from working on my EE 106A final project, a [balloon tracking robot](https://sites.google.com/berkeley.edu/b-air/). We mounted a camera on top of a TurtleBot, extracted the balloon's position, drove underneath the balloon, and used a fan to keep the balloon afloat. With this implementation, the Turtlebot's workspace was limited by the camera's field of view. If the balloon is released outside the camera's FOV, it can no longer perform detection. Having a camera track the balloon would effectively increase the camera's FOV and thus, the robot's workspace.
## Materials
* Raspberry Pi 3B+
* Pi Camera v1
* Pan-tilt servo kit
* Power supply for RPi and servos
## How It Works
### Detection
For my project, I decided to track a tennis ball, which is characterized by its green color and 2D circular shape. The algorithm works as follows:
1. Extract the image from the Pi camera as a BGR array. The camera resolution is set at (640,480), since the Raspberry Pi only has 1GB of RAM and I wanted a fairly fast frame rate.
1. Preprocessing
    1. Remove high frequency noise via Gaussian blur.
    1. Convert BGR image to HSV. Create a mask of the pixels that fall within the defined bounds of green. This gives a binary image.
    1. Perform a series of erosions and dilations to remove white noise.
1. Detection
    1. The Hough Circle Transform is used to detect circles in the filtered image. Some important parameters to tune are:
        * `minDist`: Minimum distance between circles, in pixels. The **assumption** I make is that there's only one tennis ball in the frame, so setting a large `minDist` reduces the number of circles detected and speeds up the algorithm.
        * `param2`: The smaller this value, the more false circles may be detected.
1. False Positive Rejection
     1. Find the centers of the detected circles. Calculate the distance betweent the center locations and the ball position from the previous frame.
    1. The closest center is taken to be the new detected tennis ball. The **assumption** I make here is that since the detection node is running at ~10 Hz (once every 0.1 seconds), the ball can't have traveled too far between frames. I found this to be useful in rejecting large shadows in the frame.
    1. I did explore using background subtraction to extract the moving tennis ball foreground, but with a moving camera, background subtraction did not work well since the background was constantly changing.
1. Return the center position of the tennis ball.
    1. If no ball is found in the frame, return the ball position from the previous frame.
### PID
I wrote the PID class to easily create more servo objects to accommodate future expansions of the project. There are three important methods:
    1. `update`: Takes in the error and calculates the next servo angle using PID.
    1. `normalize_servo_angle`: I define the operating range of each servo I use. This function bounds the servo angle output and prevents sending the servo an angle it can't travel to.
    1. `speed_limit`: Sets the slew rate of the servos, aka the speed limit. I noticed that my microservos, sg90 and mg90s, have trouble traveling at high speeds over a short period. In addition, moving the camera too fast blurs the image and hinders the detection node
