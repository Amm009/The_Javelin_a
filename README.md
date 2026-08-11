# The_Javelin_a
This is a work in progress between Pyro-Technic (Justin Neyman) and I. It is currently in the middle stages of development.

# The Main Idea
The Javelin(a) is inspired by the javelin missile made by the U.S Military. Don't worry, it doesn't explode (for obvious reasons). The Javelin(a) is essentially a two stage remote controlled plane. The first is a rocket stage to send it out of the launch tube and get it up to flying speed. After the first stage finishes burning, it's jettisoned. After the first stage is jettisoned, the wings fold out and a ducted fan is used to power flight. A small FPV camera is included in the nosecone of the plane to allow the pilot to fly the plane from the launch location.

![alt text](https://github.com/Amm009/The_Javelin_a/blob/main/Extra/image.png)

Below is a video I made describing the current state of the Javelin(a):

<a href="http://www.youtube.com/watch?feature=player_embedded&v=N2zlf3alyyM
" target="_blank"><img src="http://img.youtube.com/vi/N2zlf3alyyM/0.jpg" 
alt="The Javelin(a) - An Update" width="2400" height="1800" border="10" /></a>

# Plane Design
The plane model shown above was created by Justin, and will soon be undergoing a major design change. In order to keep this plane classified as a class 1 model rocket, it's maximum fully loaded weight must be 1.5kg or less. Currently it will weigh more than that. Keeping the weight down will also help it to fly better as well.

# Code
The Main Control Housing (MCH) aka the remote control is built, but the code still needs work.

Code Features Implemented:

- Radio communication between Javelin(a) and the MCH
- Secondary display functionality for all states of the Javelin(a) and MCH
- Functional states (like "arming", "firing", or "fail-safe") for the MCH

Code Features Still Being Worked on:

- Plane launch detection
- Mapping controls received from the MCH to the servos and brushless motor
- Javelin(a) states in the code are not quite working right yet
- MCH LED status light. Not implemented currently, but it's the least crucial

# Files and Code
Files and Code will be posted as they become available. You can make this at your own risk.
