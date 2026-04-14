// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonSwerveTimeControlCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // The driver's controller
  private final DriverController m_driverController = new DriverController(OperatorConstants.kDriverControllerPort);
  // The operator's controller
  private final OperatorController m_operatorController = new OperatorController(OperatorConstants.kOperatorControllerPort);

  // PathPlanner autos.
  public static HashMap<String, Command> PPAutos = new HashMap<>();

  Command m_doClimbCommand = Commands.sequence(
    // NOTE: Assumes we are close to climb entrance point and facing the correct way (away from driver station)
    // Drive right to hit tower, back until we're confident we're at the back wall, then drive forward until hook hits the bar.
    new AutonSwerveTimeControlCommand(m_robotDrive, 0, -0.06, 0, 0.75, true),
    new AutonSwerveTimeControlCommand(m_robotDrive, -0.06, 0, 0, 1, true),
    new AutonSwerveTimeControlCommand(m_robotDrive, 0, -0.06, 0, 1, true),
    new ClimberCommand(m_climber, false)
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.driveWithJoystick(m_driverController),
        m_robotDrive
      )
    );

    // Register Named Commands
    NamedCommands.registerCommand("Shoot", 
      Commands.deadline(
          Commands.sequence(
              Commands.runOnce(() -> m_intake.runRollerRPM(), m_intake),
              m_intake.agitateAuto().withTimeout(4.5),
              Commands.runOnce(() -> m_intake.stopRoller(), m_intake),
              m_intake.retractAuto()
          ),
        m_shooter.runShooterRPMCommand()
      )
    );
    NamedCommands.registerCommand("RunIntake", m_intake.extendAndRunAuto());
    NamedCommands.registerCommand("RetractIntake", m_intake.retractAuto());
    NamedCommands.registerCommand("StopIntake", Commands.runOnce(() -> m_intake.stopRoller(), m_intake));

    NamedCommands.registerCommand("RaiseClimb", new ClimberCommand(m_climber, true));
    NamedCommands.registerCommand("LowerClimb", new ClimberCommand(m_climber, false));
    NamedCommands.registerCommand("DoClimb", m_doClimbCommand);

    // Add PathPlanner autos to drop-down and build autos.
    List<String> autoNames = AutoBuilder.getAllAutoNames();
    if (autoNames != null) {
      for (String ppAutoName : autoNames) {
        PPAutos.put(ppAutoName, AutoBuilder.buildAuto(ppAutoName));
      }
    }
    SmartDashboard.putStringArray("Auto List", autoNames.toArray(String[]::new));
  }

  public void configureButtonBindings() {
    // Shooter Triggers
    // getOperatorController().runShooter.onTrue(m_shooter.runShooterRPMCommand()); // TODO: Enable after tuning RPM control.
    getOperatorController().runShooter.onTrue(m_shooter.runShooterOpenLoopCommand());
    getOperatorController().runShooter.onFalse(Commands.runOnce(() -> m_shooter.stopShooter(), m_shooter));

    // Rest of subsystems controlled in Robot::teleopPeriodic().
  }

  // GETTERS //

  DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  IntakeSubsystem getIntakeSubsystem() {
    return m_intake;
  }

  ClimberSubsystem getClimberSubsystem() {
    return m_climber;
  }

  ShooterSubsystem getShooterSubsystem() {
    return m_shooter;
  }

  DriverController getDriverController() {
    return m_driverController;
  }

  OperatorController getOperatorController() {
    return m_operatorController;
  }

  /**
   * Use this to initialize subsystems as needed. Should be called on Robot Init!
   * 
   */
  public void init() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Grab selected auto from SmartDashboard drop down menu
    String selectedAutoName = SmartDashboard.getString("Auto Selector", "");
    return PPAutos.getOrDefault(selectedAutoName, Commands.idle());
  }
}