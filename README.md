# swyftBot2026MEZ
## Getting Started
1. Follow FIRST's [Software Installation](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html) steps to setup the 2026 WPILib VS Code editor.
2. Download/Clone this repository to your computer. Make sure to save the folder outside of OneDrive.
3. Open the folder in 2026 WPILib VS Code.
4. Build the robot code using WPILib Command Palette (Ctrl+Shift+P). See [VS Code Overview](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/index.html) for help.
## Customizing the code
The code as-is should be good enough to run the SwyftBot, but there are some customizations you can/should make. Here are a couple of ideas:
### Change motor controllers
Within any subsystem, ensure your motor controllers are set to the proper one. For REV motors, these should be either SparkMax or SparkFlex.
### Remove/Update climber
A simple Climber-in-a-box subsystem has been added, but feel free to remove/update this as necessary.
### Upgrade your flywheel control
Follow the "TODO" comments in the code to set the necessary constants for velocity control of your shooter's flywheel. I recommend doing some research into PID tuning for flywheels to tackle this, but it will significantly improve the consistency of your shots!
See this [very useful article](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html).
### Upgrade your autonomous routines
Currently, the Autos.java file contains a simple "Move and Shoot" routine that can (and should) be adjusted through testing to fit your robot and your strategy.
You can also add more routines that add on to this one to do more complicated actions, such as refuel at the outpost or pass through the trench/over the bump. For more complicated routines, I recommend utilizing the distance-based control (more PID control!) for your robot's driving. This will be more consistent than time-based driving, and will be easier to use in longer routines. See the "TODO" comments placed in the code for some more info.
To take an extra step, look into [FRC PathPlanner](https://pathplanner.dev/home.html) for designing and implementing your autonomous routines. Once you tune the PID control for the drivetrain, PathPlanner becomes a very intuitive and powerful tool.
### Add vision targeting
Implementing vision targeting is a signficant boost to your robot's capabilities in teleop and autonomous. It can enable you to:
1. Shoot from various distances.
2. Aim quickly and accurately without driver input.
3. Align to outpost, trench, or tower for consistent movement.
4. Add a turret!
5. Shoot on the move!
I recommend using the plug-and-play Limelight for vision targeting. Refer to the [documentation](https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary) and [tutorials](https://docs.limelightvision.io/docs/docs-limelight/tutorials/example-projects) for more info on what you can do with vision.