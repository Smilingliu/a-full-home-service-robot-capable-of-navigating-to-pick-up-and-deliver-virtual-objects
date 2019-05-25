# a-full-home-service-robot-capable-of-navigating-to-pick-up-and-deliver-virtual-objects 
To simulate a full home service robot capable of navigating to pick up and deliver virtual objects in ros workspace.

The robot needs to have the following capabilities：

1.Initially show the marker at the pickup zone；

2.Hide the marker once the robot reaches the pickup zone；

3.Wait 5 seconds to simulate a pickup；

4.Show the marker at the drop off zone once your robot reaches it.

You need to install xterm、AMCL package before running the program.Then you need to create a new ros workspace,and then enter the following command in the terminal:

cd scripts

chmod +x home_service.sh

./home_service.sh

