# Unscented_Kalman_Filter_Highway_Project

<img src="readme_resource/highway_1.gif">

<img src="readme_resource/highway_2.gif">

<img src="readme_resource/highway_3.gif">






## Usage
Clone the Unscented Kalman Filter Highway Project package.
```
git clone https://github.com/CuteJui/Unscented_Kalman_Filter_Highway_Project.git
```
Go to the Unscented Kalman Filter Highway Project directory
```
cd /home/user/Unscented_Kalman_Filter_Highway_Project
```
Create a new directory
```
mkdir build
```
Go into the build directory
```
cd build
```
Run cmake pointing to the CMakeList.txt in the root
```
cmake ..
```
Run make
```
make
```
Run the executable
```
./ukf_highway
```

## Issues
I provide some potentail solution to the warning showing up when using cmake.
- `io features related to pcap will be disabled` \
This kind of warning is due to the missing some extra PCL libraries in the environment. Install the extra PCL libraries could fix this warning.
- `The imported target "vtkRenderingPythonTkWidgets" references the file "/usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so" but this file does not exist.` \
Install "python-vtk6" and create the link.
```
sudo apt-get update
sudo apt-get install python-vtk6
sudo ln -s /usr/lib/python2.7/dist-packages/vtk/libvtkRenderingPythonTkWidgets.x86_64-linux-gnu.so /usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so
```
- `The imported target "vtk" references the file "/usr/bin/vtk" but this file does not exist.` \
This can be fixed by the following command.
```
sudo update-alternatives --install /usr/bin/vtk vtk /usr/bin/vtk6 10
```
