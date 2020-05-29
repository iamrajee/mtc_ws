# Instruction

## Open new terminal
> To avoid conflit with ROS env variable

## Clone the workspace
`git clone https://github.com/iamrajee/mtc_ws.git`

## Move to folder
`cd mtc_ws`

## Build the workspace
`catkin_make`

## Open one more new terminal run below command in both terminals:
> Lets sets the parameter specific to ros version (here melodic), must do once in each terminal\
`source /opt/ros/melodic/setup.bash `
> Lets sets the parameter specific to ros workspace (here mtc_ws), must do everytime after building the workspace\
`source devel/setup.bash`

## Run the demo
Terminal 1: `roslaunch mtc_pickplace jaco_demo.launch`\
Terminal 2: `roslaunch mtc_pickplace pickplce.launch`

