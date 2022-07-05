# ROB 535 Control Project: Racing on a Pre-Defined Map with Unknown Obstacles #

### Introduction ###
The control project of controlling a bicycle model mainly contains two parts as follows.

* Task 1: We are required to design a controller for the system to get from the beginning to the end of a pre-defined track as rapidly as possible; 
* Task 2: We need to develop a control design algorithm (which may or may not modify the controller constructed in the first task) to avoid obstacles that are known only at run-time.
  
More detailed requirements can be found in ROB535_Control_Project.pdf

### Run ###
All code can be executed in MATLAB environment:

* Task1, open folder ./part1_final and run main_p1.m. 

* Task2, open folder ./part2_final and run forwardIntegrate.m file. Our main contribution is the ROB535_ControlProject_part2_Team26.m function.

### Result ###

* Task1: Feeding our designed control input ((ROB535_ControlProject_part1_input.mat) into the system function(forwardIntegrateControlInput), we can get the trajectory information. We will display the trajectory path using the red line.

* Task2: The result is info_result.png and .png


### Contact ###

* Jiachen Jiang jiachenj@umich.edu
