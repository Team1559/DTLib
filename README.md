# DTLib
DTLib is an extension of [WPILib](https://github.com/wpilibsuite/allwpilib) designed by FRC Team 1559 for the [FIRST Robotics Competition](http://firstinspires.org/robotics/frc). It adds features that we felt were lacking, as well as improving code reusability and maintainability. The primary motivation behind this project is to ensure the software team can get up and running before the robot is fully assembled (or even designed).

## License

This library is licensed under the [GNU Public License V3](https://github.com/Team1559/DTLib/blob/master/LICENSE). However, not all dependencies are licensed in the same way. Notably, WPILib uses their own custom license (which can be found [here](https://github.com/wpilibsuite/allwpilib/blob/main/LICENSE.md)). To summarize, the licenses state that users are free to copy, modify and redistribute our code as much as they like, so long as we and our dependencies receive attribution for our work with the original licenses and copyrights included.

## Languages

Currently, only WPILib's `wpilibj` is supported in Java (including the commands framework). There are no current plans to add support for another language; however, if it proves useful to other people, we might port it in the future.

## Contributing

Contributions are always welcome, so feel free to open an issue or pull request if changes need to be made. Any submitted code must pass all automated checks (including a project build) before it will be reviewed by a team member.

## Building

The project can be built as-is using the included Gradle wrapper, and all required dependencies will be downloaded before compiling. The sole requirement is a configured Java JDK version capable of running the wrapper with `JAVA_HOME` set up in the environment.

* *Building on Unix systems:* `./gradlew build`
* *Building on Windows systems:* `gradlew build`

## Usage

**Units**
Unless explicitly specified in documentation, all units used within this library are *metric*. Typical units utilized include the following:
* Frequency: *hertz*
* Time: *seconds, microseconds, robot program cycles*
	* *A typical FRC robot program runs at 50 Hz, or one cycle every 0.02 seconds*<p>
* Linear displacement: *meters*
* Angular displacement: *degrees, radians, revolutions*
* Linear velocity: *meters per second*
* Angular velocity: *degrees per second, revolutions per minute*
* Linear acceleration: *meters per second squared*
* Angular acceleration: *degrees per second squared*<p>
* Mass: *kilograms*
* Torque: *Newton-meters*
* Voltage: *volts*
* Current: *amps*

## Authors

*Students*

[@MathNerd28](https://github.com/MathNerd28) - Alexander Bhalla, [Team 1559](https://github.com/Team1559/)

*Mentors*

[@mmarchetti](https://github.com/mmarchetti) - Mike Marchetti, [Team 1559](https://github.com/Team1559/)
