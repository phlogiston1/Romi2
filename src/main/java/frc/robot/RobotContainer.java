// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ChezzyDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import static frc.robot.Constants.DriveConstants.*;

import java.util.List;

import static frc.robot.Constants.AutoConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick drvJoy = new Joystick(0);

  private final Drivetrain romiDrivetrain = new Drivetrain();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(romiDrivetrain);
  //private final ChezzyDrive driveCommand = new ChezzyDrive(romiDrivetrain, drvJoy);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //romiDrivetrain.setDefaultCommand(driveCommand);
  }

/**
   * Generate a trajectory following Ramsete command
   * 
   * This is very similar to the WPILib RamseteCommand example. It uses
   * constants defined in the Constants.java file. These constants were 
   * found empirically by using the frc-characterization tool.
   * 
   * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
   */
  private Command generateRamseteCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(ksVolts, 
                                       kvVoltSecondsPerMeter, 
                                       kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(kMaxSpeedMetersPerSecond, 
                             kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.35, 0.07),
            new Translation2d(0.47, 0.41),
            new Translation2d(0.20, 0.43),
            new Translation2d(0,0.29),
            new Translation2d(-0.20, 0.33),
            new Translation2d(-0.12,0.6)
        ),
        new Pose2d(0.16, 0.64, Rotation2d.fromDegrees(0)),
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        romiDrivetrain::getPose,
        new RamseteController(kRamseteB,kRamseteZeta),
        new SimpleMotorFeedforward(ksVolts,kvVoltSecondsPerMeter),
        kDriveKinematics,
        romiDrivetrain::getWheelSpeeds,
        new PIDController(kPDriveVel, 0, 0),
        new PIDController(kPDriveVel, 0, 0),
        romiDrivetrain::tankDriveVolts,
        romiDrivetrain);

    romiDrivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(() -> romiDrivetrain.resetOdometry(exampleTrajectory.getInitialPose()), romiDrivetrain)
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> romiDrivetrain.tankDriveVolts(0, 0), romiDrivetrain));
  } 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    // Example of how to use the onboard IO
    // Setup SmartDashboard options
    //m_chooser.setDefaultOption("Ramsete Trajectory", generateRamseteCommand());
    
    //SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return generateRamseteCommand();
    

  }
}
