Modern Player
------------
As we know,ROS is really popular nowadays.However，Player is a perfect substitute in robots for those who dislike ROS，
or whose demands can't be fullfilled by ROS.This is a modern version of Player/Stage with modern drivers such as:
Rplidar S1 laser scan,Cartographer by Google etc.

Player - one hell of a robot server
----------------------------------
This is the Player robot device server, developed by volunteer contributors,
with help from many other places, and released under the GNU Lesser General
Public License (LGPL).  Some of the code is only available under the GPL,
see individual libraries and source files for details.

The Player homepage is:

  https://playerproject.github.io/


Installation
------------
Read INSTALL for more detailed instructions.  For most people, the following
sequence will suffice:

```
  $sudo apt-get install libgeos++-dev libeigen3-dev
  $ mkdir build  
  $ cd build/  
  $ cmake ../  
  $ make install  
```

To configure the build, do:

```
  $ ccmake ../
```
To run a drivers,ie,Rplidar S1, do:
```
  $ player rplidar.cfg
```
