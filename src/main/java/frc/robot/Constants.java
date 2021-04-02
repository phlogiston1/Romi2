// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double ksVolts = 0.929;
    public static final double kvVoltSecondsPerMeter = 6.33;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0389;

    public static final double kPDriveVel = 0.085;

    public static final double kTrackwidthMeters = 0.142072613;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double      DRIVETRAIN_KS       = 0,
                                    DRIVETRAIN_KV       = 0.03,
                                    DRIVETRAIN_KA       = 1,
                                    DRIVETRAIN_VEL_KP   = 0.03,
                                    DRIVETRAIN_VEL_KI   = 0.02,
                                    DRIVETRAIN_VEL_KD   = 0,
                                    DRIVETRAIN_POS_KP   = 0, 
                                    DRIVETRAIN_POS_KI   = 0, 
                                    DRIVETRAIN_POS_KD   = 0; 
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
