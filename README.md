# DTLib
DTLib is an extension of [WPILib](https://github.com/wpilibsuite/allwpilib) designed by FRC Team 1559 for the [FIRST Robotics Competition (FRC)](http://firstinspires.org/robotics/frc). It adds features that we felt were lacking, as well as improving code reusability and maintainability. The primary effort behind this project is to ensure the software team can get up and running well before any parts of the robot are actually constructed.

## License
This library is licensed under the [GNU Public License V3](https://github.com/Team1559/DTLib/blob/master/LICENSE). However, it must be noted that not all dependencies are licensed in the same way. Notably, WPILib uses their own custom license (which can be found [here](https://github.com/wpilibsuite/allwpilib/blob/main/LICENSE.md)). In short, users are free to copy, modify and redistribute this code as they like, so long as we receive attribution for our work.

## Languages
Currently, only WPILib's `wpilibj` is supported in Java (including the commands framework). There are no current plans to add support for another language; however, if it proves useful to other people, we might port it in the future.

## Contributing
Contributions are always welcome, so feel free to open an issue or pull request if changes need to be made. Any submitted code must pass all automated checks (including a complete library build) before it will be reviewed by a team member.

## Building
The project can be built as-is using the included Gradle wrapper, and all required dependencies will be downloaded before compiling. The sole requirement is a configured Java JDK version capable of running the wrapper with `JAVA_HOME` set up in the environment.

*Building on Unix systems:* `./gradlew build`
*Building on Windows systems:* `gradlew build`

## Authors
**From [Team 1559](https://github.com/Team1559/)**
[@MathNerd28](https://github.com/MathNerd28) - Alexander Bhalla

