# Realizing  Simultaneous  Lane  Keeping  and  Adaptive  Speed  Regulation on  GRITBOTS  of  Robotarium

## Project Overview
In this project we try to accomplish two goals - First, we survey the literature relating to the Non linear contro theory using Control Lypanouv Functions and Control Barrier Functions in order to summarize this newly emerging field. Second, we try to implement a few different controllers onto the Robatarium testbed for adaptive speed regulation and lane keeping. We ran two main simulations. The first uses Barrier Certificates and the second uses the properties of Control Lyapunov Functions and Control Brrier Functions for simultaneous achievement of twin objectives of performance and safety guarantees.
We designed a way point controller for tracking a prespecified trajectory with a fixed distance metric between leader and follower cars. After implementing this project on the Robotarium Test Bed we designed the more involved controller which uses a quadratic program for mediating the conflicting requirements of performance, stabilty and safety guarantees.
This framework is referred to as CLF-CBF-QP.

## Implementation
The implementation of the CLF-CBF-QP controller was written in MATLAB, tested on the Robotarium simulator, and then tested on the actual testbed. 

The actual code is available in this Github Repo. (Please click on the image below)
<a href="https://github.com/warrendeng/206b-Final-Proj/tree/master/code">
 <img src="media/code.png" style="width: 60%; display: block; margin: auto;" />
</a>


## Videos of Results

Here we have a video of the Barrier Certificate Algorithm being run on the actual Robatarium testbed.
<video width="640" height="480" controls>
  <source src="media/robotarium_video.mp4" type="video/mp4">
</video>
Here we have a video of the CLF-CBF-QP algorithm being tested for tracking. 
<video width="640" height="480" controls>
  <source src="media/tracking.mp4" type="video/mp4">
</video>

## Team
#### Gyanendra Tripathi 
<a href="https://github.com/warrendeng/206b-Final-Proj/tree/master/code">
 <img src="media/self.png" style="width: 60%; display: block; margin: auto;transform:rotate(90 deg);" />
</a>
Gyanendra is a graduate student of Public Policy (GSPP) with over 2 decades of experience in implementation and formulation of public policies in India. He has successfully completed EE 206A in last fall and in this semester besides pursuing EE 206B he is also learning 'Introduction to Machine Learning CS-289A'. He did BE in Electrical Engineering from IIT Roorkee(1996) and MTech in Power Electronics from IIT Kanpur(1998) before undertaking his career in civil service of India.
