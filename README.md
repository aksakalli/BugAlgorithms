Here is the bug0, bug1 and bug2 motion planning algorithms that implementation on Webots simulation environment. Pioneer robot is used that equipped with following sensors:

* 2x GPS
* 7x infra-red front side sensor
* 1x Kinect

All of the three algorithms were implemented in three individual projects with listed sensors on pioneer robot. These motion planning algorithms' aim is reaching the target. Ported tree is target in simulation environment. Walls are positioned like the example in lecture slides.

Kinect detects the obstacle and the robot starts to follow the obstacle. During following, infra-red sensors in the each side control the distance through the obstacle.

![bug motion planning angel](https://raw.github.com/aksakalli/BugAlgorithms/assets/1.jpg)

To determine the position and angle of the target two GPSs are used. Back GPS to center GPS angle is defined as "currentAngle", back GPS to target is defined as "targetAngle".

When the robot in "GO_TO_TARGET" mode, it tries to reduce the difference of two angles. Right wheel speed is increased and left wheel speed is decreased when the target angle is bigger than current angle opposite way is done for smaller situation. This progress continues up to threshold value. If the difference of two angles is smaller than that value, both wheels are set to normal speed.

![increased angel](https://raw.github.com/aksakalli/BugAlgorithms/assets/2.jpg)

##Bug0 Algorithm
- Head toward goal
- Follow obstacles until you can head toward the goal again
- Continue

[![Bug 0 youtube video](http://img.youtube.com/vi/C6GmD4qS3bs/1.jpg) Watch Youtube video](http://www.youtube.com/watch?v=C6GmD4qS3bs)


It was not defined as a monotonically right or left turning algorithm. In this approach, the pioneer robot turns in direction that has long distance to obstacles. It follows the wall until the side sensors not detect an obstacle and then it tries to go the target. When it reaches the target, the main while loop is over and robot stops.

##Bug1 Algorithm
- Head toward goal
- If an obstacle is encountered, circumnavigate it and remember how close you get to the goal
- Return to that closest point (by wall-following) and continue

[![Bug 0 youtube video](http://img.youtube.com/vi/iJWULA_gIy8/1.jpg) Watch Youtube video](http://www.youtube.com/watch?v=iJWULA_gIy8)

###Robot has two modes

**GO_TO_TARGET :** Robot tries to go to target point with calculating the angles. If it detects an obstacle stores the initial minimum distance values that coordinate point, time and distance to target.

**FOLLOW_OBSTACLE :** Robot follows the obstacle and stores minimum distance point for its route. When it is at the closed point for a second time, it switches to "GO_TO_TARGET" mode. How can the robot understand that this is first or second time that it passes through the minimum distance point? Robot also saves the time period of reaching the minimum point. If there is more than three second differences between reaching the short distance point, it must be second time. Otherwise, robot always switches to "GO_TO_TARGET" mode because it reaches the smallest point for every step during coming closer to target.

##Bug2 Algorithm
 
- Head toward goal on the m-line
- If an obstacle is in the way, follow it until you encounter the m-line again closer to the goal.
- Leave the obstacle and continue toward the goal

[![Bug 0 youtube video](http://img.youtube.com/vi/Z5-TBsKPCF0/1.jpg) Watch Youtube video](http://www.youtube.com/watch?v=Z5-TBsKPCF0)

The robot calculates its initial angle to target. In "GO_TO_TARGET" mode, it tries to reach the target. If it detects an obstacle, it switches to "TURN_RIGHT_FOLLOW" or "TURN_LEFT_FOLLOW" mode. Actually they are making the same things accept turning direction. In following mode robot follows the obstacle until reaching the same angle that in the initial step.

![bug2 motion planning path](https://raw.github.com/aksakalli/BugAlgorithms/assets/3.jpg)

Explanation of algorithms are taken from [here](http://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf)