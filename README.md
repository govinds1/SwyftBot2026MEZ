# swyftBot2026MEZ
## Getting Started
1. Follow FIRST's [Software Installation](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html) steps to setup the 2026 WPILib VS Code editor.
2. Fork this repository to make a new repository for your team! Then clone/download the code to your laptop, making sure to save the folder outside of OneDrive. See [Git introduction](https://docs.wpilib.org/en/stable/docs/software/basic-programming/git-getting-started.html).
3. Open the project folder in 2026 WPILib VS Code.
4. [Build and deploy](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/deploying-robot-code.html) the robot code using WPILib Command Palette (Ctrl+Shift+P).  

I highly recommend familiarizing yourself with [WPILib's Programming Basics](https://docs.wpilib.org/en/stable/docs/software/basic-programming/index.html) as a launch point into FRC programming.  
## Customizing the code
The code as-is should be good enough to run the SwyftBot, but there are some customizations you can/should make. Here are a few ideas:
### Change motor controllers
Within any subsystem, ensure your motor controllers are set to the proper one. For REV motors, these should be either SparkMax or SparkFlex.
### Remove/Update climber
A simple Climber-in-a-box subsystem has been added, but feel free to remove/update this as necessary.
### Upgrade your flywheel control
Follow the "TODO" comments in the code to set the necessary constants for velocity control of your shooter's flywheel. I recommend doing some research into PID tuning for flywheels to tackle this, but it will significantly improve the consistency of your shots!  
See [Tuning a Flywheel Velocity Controller](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html). PID control is a widely used concept in robotics, even outside of FIRST!
### Upgrade your autonomous routines
This code uses [FRC PathPlanner](https://pathplanner.dev/home.html) for designing and implementing your autonomous routines. PathPlanner can be an extremely intuitive and powerful tool for creating autonomous paths and building entire autos. I highly recommend reading through the PathPlanner documentation and examples to familiarize yourself with the usage, and then dive into this code to see how it is implemented. I also recommend installing PathPlanner and creating paths and autos of your own!
### Add vision targeting
Implementing vision targeting is a signficant boost to your robot's capabilities in teleop and autonomous. It can enable you to:
1. Shoot from various distances.
2. Aim quickly and accurately without driver input.
3. Align to outpost, trench, or tower for consistent movement.
4. Add a turret!
5. Shoot on the move!  

I recommend using the plug-and-play Limelight for vision targeting. Refer to the [documentation](https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary) and [tutorials](https://docs.limelightvision.io/docs/docs-limelight/tutorials/example-projects) for more info on what you can do with vision.