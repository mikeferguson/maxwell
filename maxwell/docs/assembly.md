# Maxwell Assembly Notes

## Part Modifications

The BaneBots hubs are only available in Metric, the motor shafts however are
English. The 6mm bore on the hubs will need to be slightly enlarged to a 1/4"
bore.

One of the pieces of 8020 rail will need to be cut to 14" long for use as
the upper piece of Maxwell's torso.

The 20" track-style linear actuator has no feedback. Feedback has been added
by putting an optical flag on the threaded rod, providing a low resolution
encoder. The flag is 3D printed, as is the mount for the photointerrupter.

TODO: add pictures of linear actuator encoder.

# Assembling the Power Board

The first step in assembling the power board is to install 0.1" headers. Cut
off 5-pin lengths of header, removing every other pin so that there are only
3 pins. Solder the headers in:
![Power Board 1](https://raw.githubusercontent.com/mikeferguson/maxwell/indigo-devel/maxwell/docs/power_board_1.jpg)

The second step is to install the terminal blocks:
![Power Board 2](https://raw.githubusercontent.com/mikeferguson/maxwell/indigo-devel/maxwell/docs/power_board_2.jpg)

A schematic of the power board can be downloaded as a
[PDF](https://raw.githubusercontent.com/mikeferguson/maxwell/indigo-devel/maxwell/docs/powerboard.pdf).

## Wiring

The following diagram is a complete view of the wiring in Maxwell:

![Wiring Diagram](https://raw.githubusercontent.com/mikeferguson/maxwell/indigo-devel/maxwell/docs/wiring_diagram.png)

The above wiring diagram can also be downloaded as a
[PDF](https://raw.githubusercontent.com/mikeferguson/maxwell/indigo-devel/maxwell/docs/wiring.pdf).

The power switch is wired up so that in one position, it conducts from the battery
to the power board, and in the other position it will conduct from the power jack
to the power board. This allows users to plug the power supply in and then switch
to external power without turning the robot off.

The power jack is located directly below the power switch, while the charge plug
is located on the other side of the robot. The image below shows how the switch
is wired:

![Power Switch](https://raw.githubusercontent.com/mikeferguson/maxwell/indigo-devel/maxwell/docs/power_switch.jpg)

Most of the wiring can be installed while only the base is assembled. When the
panel of the base is flipped down, it will look something like:

![Wiring Diagram](https://raw.githubusercontent.com/mikeferguson/maxwell/indigo-devel/maxwell/docs/wiring_view.jpg)

In this picture, the power board is on the left, the Etherbotix with motor
driver is in the center, and the torso lift motor driver is on the right.
The power cables coming from the power switch in the back of the robot are
wired into the 2-pin black header on the power board.

The Etherbotix supports split power supplies for logic and motors/servos.
A single ground wire runs from the power board to the Etherbotix.
The power board routes 12V directly to the Etherbotix logic supply, while the
motor/servo power is passed through the runstop. The auxillary power output
on the Etherbotix (which is connected to motor/servo power) is routed to the
motor driver board which controls the base motors. The torso lift motor driver
is connected to a servo header for power.

In the image above, the 19V power cable is not installed in the 2-pin orange
terminal block on the power board.

In this configuration, measured currents include:

 * Servo Current - measures the current used by all servos, as well as the
   torso lift.
 * Auxilary Current - measures the current used by the base motors.

## Assembly

The mobile base is constructed of several lasercut ABS panels, held together
with 4-40 screws and right angle brackets (Digikey 621K-ND). Note that the
brackets are not symmetrical, check that they are installed correctly (the
holes in the laser cut components are made asymmetrical to match the bracket
installation direction). 1/4" long screws are used everywhere except for holding
the top down, where we use 3/8" long screws.
 
![Base Top View](https://raw.githubusercontent.com/mikeferguson/maxwell/indigo-devel/maxwell/docs/top_view.jpg)

The "upright holder" piece is used as a retaining socket for the 8020 upright.
This piece is bolted to the bottom of the base using 1/2" long 4-40 screws.

The caster is attached to the top plate of the base using 1/4-20 standoffs,
and pan head bolts (these are actually extras from the bag of 8020 components,
but have a large head and the black finish matches the ABS).

TODO: assembling the torso lift

TODO: assembling the arm

TODO: assembling the neck and head
