# AntColony
4M20

The ant class created contains the properties of the braitenberg vehicle and additional properties such as the local pheremon matrix (see additional properties under properties to see all). It also contains two functions - the class initializer and the controller. 

PROPERTIES
The key properties that enable the 'ant braitenberg vehicle' to perform as required are that it updates its properties to keep a historic record of its position over tiume. In addition, it has features such as the gain for proportional control, variable noise to enable random movements and variables that enable each ant object to decide which direction it is going to head to next in each time step. 

METHODS
The initializer enables the 'ant braitenberg vehicle' eg ant1 = ant([x,y,theta],pher) object to be initialized at a starting position in the maze. 

The controller eg ant1.controller(Surface,time) takes in the potential surface and time step to calculate its final position at the end of the time step. 
1. It is able to convert from a continuous position to a discrete position in a grid that has the generated potential field. 
2. Identify which region in a 3x3 grid within the maze potential it is on.
3. Adds noise to its observation of the 3x3 maze potential. 
4. Identifies the direction it was heading towards in its previous time step (and continues heading there until it reaches a new region)
5. Identifies the lowest step in its noisy observation of its surroundings and chooses this direction to head to (gradient decent). 
6. Calculates the required wheel_omega required to head to its desired direction with a proportional controller.
7. Updates its postion based on the wheel_omega
8. Records its steps



