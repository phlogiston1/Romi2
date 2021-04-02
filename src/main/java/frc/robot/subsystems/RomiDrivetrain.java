// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.RomiGyro;

import static frc.robot.Constants.DriveConstants.*;
//is it working?

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm
  private static final double kWheelDiameterMeter = 0.07;

  private double prevLDist = 0, prevRDist = 0;
  // private static double lDriftAccumulator = 0;
  // private static double rDriftAccumulator = 0;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);
  public DifferentialDrive dDrive = new DifferentialDrive(m_leftMotor, m_rightMotor); 
  private final RomiGyro gyro = new RomiGyro(); 
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  private final DifferentialDriveOdometry odometry;
  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_leftMotor.enableDeadbandElimination(true);
    m_rightMotor.enableDeadbandElimination(true);
    resetEncoders();

    odometry = new DifferentialDriveOdometry(gyro.getHeading());
  }
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
    dDrive.feed();
  }
  
  public void arcadeDrive(double speed, double turn) {
    velocityDrive((speed - turn)*20,(speed + turn)*20);
  }
  private SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(DRIVETRAIN_KS,DRIVETRAIN_KV,DRIVETRAIN_KA);
  private SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(DRIVETRAIN_KS,DRIVETRAIN_KV,DRIVETRAIN_KA);
  private PIDController lPidController = new PIDController(DRIVETRAIN_VEL_KP, DRIVETRAIN_VEL_KI, DRIVETRAIN_VEL_KD);
  private PIDController rPidController = new PIDController(DRIVETRAIN_VEL_KP, DRIVETRAIN_VEL_KI, DRIVETRAIN_VEL_KD);
  
  public void velocityDrive(double lSpeed, double rSpeed){
    
    double lpid = lPidController.calculate(Units.metersToInches(-getLeftVelocity()),lSpeed);
    double lffd = leftFeedforward.calculate(Units.metersToInches(-getLeftVelocity()));
    double rpid = rPidController.calculate(Units.metersToInches(getRightVelocity()),rSpeed);
    double rffd = rightFeedforward.calculate(Units.metersToInches(getRightVelocity()));
    
    SmartDashboard.putNumber("l velocity setpoint", lSpeed);
    SmartDashboard.putNumber("r velocity setpoint", rSpeed);
    SmartDashboard.putNumber("l velocity", getLeftVelocity());
    SmartDashboard.putNumber("r velocity", getRightVelocity());

    tankDrive(-(lpid + lffd), -(rpid + rffd));
  }

  double kP = DRIVETRAIN_POS_KP, kI = DRIVETRAIN_POS_KI, kD = DRIVETRAIN_POS_KD;
  double li = 0, ri = 0;
  double prevREr = 0, prevLEr = 0;
  public void positionDrive(double leftPosition, double rightPosition){
    double lError = Units.metersToInches(getLeftDistanceMeter()) - leftPosition;
    double rError = Units.metersToInches(getRightDistanceMeter()) - rightPosition;

    double lp = kP * lError;
    double rp = kP * rError;

    li += kI;
    ri += kI;

    double ld = (lError - prevLEr) * kD;
    double rd = (prevREr - rError) * kD;

    tankDrive(-(lp + li + ld), rp + ri + rd);

    prevLEr = lError;
    prevREr = rError;
  }

  public void tankDriveVolts(double lVolts, double rVolts){
    m_leftMotor.setVoltage(lVolts);
    m_rightMotor.setVoltage(-rVolts);
    dDrive.feed();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }


  private static final double backlashCorrectionDistance = 0.0075; //meters
  private double backlashCorrectionCurrentRange = 0;
  private boolean backlashDirection = true; //true = val > prev val.
  private boolean correcting = false;
  private double correctBacklash(double position, double prevPosition) {

    boolean newBacklashDirection = backlashDirection;
    if(position != prevPosition) newBacklashDirection = position > prevPosition;
    SmartDashboard.putBoolean("bd", newBacklashDirection);
    if(newBacklashDirection != backlashDirection) {
      correcting = true;
      backlashCorrectionCurrentRange = 0;
    }
    if(backlashCorrectionCurrentRange > backlashCorrectionDistance) correcting = false;
    SmartDashboard.putBoolean("ct", correcting);
    if(correcting){
      backlashCorrectionCurrentRange += Math.abs(position - prevPosition);
    }
    backlashDirection = newBacklashDirection;
    return backlashDirection ? position + backlashCorrectionCurrentRange : position - backlashCorrectionCurrentRange;
  }

  
  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  // public double getLeftDistance(){
  //   double dist = Units.inchesToMeters(getLeftDistanceInch());
  //   double corrected = correctBacklash(dist, prevLDist);
  //   prevLDist = dist;
  //   return corrected;
  // }

  // public double getRightDistance(){
  //   double dist = Units.inchesToMeters(getRightDistanceInch());
  //   double corrected = correctBacklash(dist, prevRDist);
  //   prevRDist = dist;
  //   return corrected;
  // }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  public double getLeftVelocity(){
    return m_leftEncoder.getRate();
  }
  public double getRightVelocity(){
    return m_rightEncoder.getRate();
  }
  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
/**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(gyro.getHeading(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently estimated pose of the robot.
   * @return The pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getHeading());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly
   * @param maxOutput The maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    dDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot
   * @return The robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getHeading().getDegrees();
  }

  /**
   * Returns the turn rate of the robot
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRateZ();
  }
}
