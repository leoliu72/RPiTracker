# RPiTracker
<p float="left">
  <img src="/images/demo.gif" width="270" />
  <img src="/images/setup.jpg" width="270" height="480" /> 
</p>

## File Structure
`detection.py` is the Detection class used to find the center of the ball.
`pid.py` is the PID class used to calculate the appropriate servo angle.
`tracking.py` keeps everything together in a nice loop.
## Running the code
First, set up OpenCV in your work environment. I have mine in a virtual environment, following [this tutorial](https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/) from Adrian Rosebrock of PyImageSearch.

Then, run `python tracking.py`
## Motivation
The idea for this project came from working on my EE 106A final project, a [balloon tracking robot](https://sites.google.com/berkeley.edu/b-air/). We mounted a camera on top of a TurtleBot, the camera extracted the balloon's location, and the robot drove underneath the balloon, using a fan to keep the balloon afloat. The problem with this implementation was that the robot's workspace was limited by the field of view of the camera. Toss the balloon out of the camera's range, and you can no longer perform detection. Having a moving camera would effectively increase the camera's field of view and thus, the robot's workspace.
