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
