>create a empty catkin package
>create the source code of this package for a basic ros node (use existing code to save time and adapt it to your purposes)
>Identify the topics to which your node must subscribe. Use "rostopic list | grep" "rostopic type" "rosmsg show", etc
>Register that topics for subscription. Include the respective headers (each topic/service dependency must have its header)
>Create the respective callbacks for each subscription (no code inside for the moment)(take advantage of examples)
>Identify the topics that your application may publish (this depends on the application, if already exist identify them)
>Register the publishers (take code from examples)
>Modify the package.xml to include the dependencies of your node (each package dependency should be put in "build_depend" and "run_depend")
>Modify the CMakeList.txt to include the dependencies of your node. (at least include your dependencies in "find_package" "catkin_package")
>Compile the node. Solve any problem about the compilation (library dependencies, include directories, etc)
>Run the node and check connections (use rosnode info, rqt_graph, rostopic info, etc)
>Identify the parameters of your application
>Program parameters reading (take an example of how to do it)
>Program the code of your application (normally in each callback and/or in the main loop)
>Create a launch file that launch the nodes necessary, defines parameters and remap any topic names
>Test it 
