# matlab-simulink-s-function
Here are some examples of how to create custom S-functions to interface C++ or C code with the Simulink coder. This allows users to compile their Simulink models with third-party software, which is not possible otherwise.
# Structure
The code follows typical C++ project structure with an include dir, src dir, and an optional build file. In this example we will use *CMake* to ease the build process.
```
├── CMakeLists.txt
├── include
│   └── optical_flow_uav_velocity.hpp
├── include_directories.txt
├── LICENSE
├── link_libraries.txt
├── README.md
├── source_files.txt
├── src
│   ├── matlab_simulink_s_function.cpp
│   └── optical_flow_uav_velocity.cpp
└── test_s_function.slx

```
# Usage
1. Prepare source and include files for your custom ```class``` and place them in the appropriate directories.
2. Edit ```src/matlab_simulink_s_function.cpp``` and include your custom headers.
3. Currently there are two ways to instantiate your custom ```class``` objects and access their methods.
   1. Using persistent memory
   2. Using static objects
4. You should specify this by ```#define USE_PERSISTENT_MEMORY``` statement in the top of the ```s_function``` file.
5. There are ```4``` important functions and  ```1``` optional function that needs to be implemented
   1. ```mdlInitializeSizes( SimStruct *S)```
      1. Here, we specify the number of inputs and outputs and set the sizes of the input and output signals.
      2. Furthermore, you can also specify the number of discrete sample times this s-function has, and set the number of persistent memory objects
   2. ```mdlInitializeSampleTimes(SimStruct* S) ```
      1. In this function, we set the sample time(s) of this s-function. Typically we use inherited sample time via ```ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);```
   3. ```mdlStart(SimStruct* S)```
      1. This very important function allows us to create instances of the custom class and is only executed once ( if ```#define MDL_START``` is defined). Here we can create new instances with dynamic storage duration, and store them in the persistent workers or static containers or objects created we created in ```mdlInitializeSizes```.
   4. ```mdlOutputs(SimStruct* S, int_T tid)```
      1. The main function of your s-function. This is executed every simulation step, and within this block we can access ptrs to input signal, output signal, signal size info, parameter info, etc. Example implementation is provided in the ```src/matlab_simulink_s_function.cpp```
   5. (optional) ```mdlTerminate(SimStruct* S)```
       1. This is the cleanup code and should delete objects created via ```new``` statement in ``` C++ ```
6. The ```SimStruct *S``` is the window to the Simulink execution state and static parameters. Through the ```SimStruct``` we can access pointers to the ```inputs```, ```outputs```, and ```parameters```.
7. Examples of how to leverage the ```SimStruct``` can be found here [simstruct examples](https://www.mathworks.com/help/simulink/sfg/simstruct_introduction.html)
8. Finally, edit ```CMakeLists.txt``` and modify ```set(CUSTOM_PACKAGES OpenCV
    ) ``` with packages required by your source code. For example ```set(CUSTOM_PACKAGES pkg1
   pkg2 ...
    ) ```. Then ```set(SRCS ${CMAKE_SOURCE_DIR}/src/matlab_simulink_s_function.cpp ${CMAKE_SOURCE_DIR}/src/optical_flow_uav_velocity.cpp)
set(INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include")```. For your source if the name is different, then modify them accordingly.
9. Create a ```build``` directory and run ```cmake```
    1. ```bash
       mkdir build && cd build
       cmake .. -DMATLAB_BIN_DIR=/path/to/my/matlab/binary/dir
       ```
   2. If everything is OK the configuration will finish and your s-function can be found in the build directory
10. To use it in a simulink model, an example file is provided ```test_s_function.slx```

# Support
If you find this repo helpful, please ⭐ star the repo to show your support!
[![GitHub stars](https://img.shields.io/github/stars/AmalDevHaridevan/matlab-simulink-s-function.svg?style=social&label=Star)](https://github.com/AmalDevHaridevan/matlab-simulink-s-function/stargazers)
