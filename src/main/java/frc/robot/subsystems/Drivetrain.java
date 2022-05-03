/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PIDConstants;
import frc.robot.constants.PortConstants;
import frc.robot.constants.RobotDimensionConstants;
import frc.robot.constants.TrajectoryConstants;
import frc.robot.constants.VoltageConstants;

/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase {

  public WPI_TalonFX[] leftTalons = new WPI_TalonFX[3];
  public WPI_TalonFX[] rightTalons = new WPI_TalonFX[3];

  public DifferentialDrive diffDrive;
  private TalonFXSensorCollection leftEncoder;
  private TalonFXSensorCollection rightEncoder;

  public DifferentialDriveKinematics kinematics = 
  new DifferentialDriveKinematics(RobotDimensionConstants.ROBOT_WIDTH_CENTER_WHEELS /39.37); //39.37 inches per meter
  DifferentialDriveOdometry odometry;

  AHRS ahrs;
  public double accelerationDueToGrativy = 9.80665; //in m/s^2

  public Drivetrain(){

    try {
      ahrs = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    odometry = new DifferentialDriveOdometry(getHeading());

    for(int i=0; i<PortConstants.LEFT_MOTORS_IDS.length; i++){
      if(i==0){
        leftTalons[i] = configTalons(PortConstants.LEFT_MOTORS_IDS[i], true, false);
      }
      else{
        leftTalons[i] = configTalons(PortConstants.LEFT_MOTORS_IDS[i], false, false);
        leftTalons[i].follow(leftTalons[0]);
      }
    }

    for(int i=0; i<PortConstants.RIGHT_MOTORS_IDS.length; i++){
      if(i==0){
        rightTalons[i] = configTalons(PortConstants.RIGHT_MOTORS_IDS[i], true, false);
        
      }
      else{
        rightTalons[i] = configTalons(PortConstants.RIGHT_MOTORS_IDS[i], false, false);
        rightTalons[i].follow(rightTalons[0]);
      }
    }

    diffDrive = new DifferentialDrive(leftTalons[0], rightTalons[0]);
    //diffDrive.setSafetyEnabled(false); //take that lila

    leftEncoder = leftTalons[0].getSensorCollection();
    rightEncoder = rightTalons[0].getSensorCollection();
		
  }
  
  public WPI_TalonFX configTalons(int _canId, boolean isMaster, boolean isInverted){
    WPI_TalonFX talon = new WPI_TalonFX(_canId);
    talon.configFactoryDefault();
    talon.setInverted(isInverted);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0, PIDConstants.kTimeoutMs);

    if(isMaster){
      talon.configOpenloopRamp(VoltageConstants.openDriveVoltageRampRate);
    }

    return talon;
  }

  public void setDrivetrainNeturalMode(NeutralMode _mode){
    leftTalons[0].setNeutralMode(_mode); //the 0 index is the master motor
    rightTalons[0].setNeutralMode(_mode);
  }

  public void stopDrivetrain(){
    leftTalons[0].set(ControlMode.PercentOutput, 0); //the 0 index is the master motor
    rightTalons[0].set(ControlMode.PercentOutput, 0);
  }

	@Override
  public void periodic(){
    odometry.update(ahrs.getRotation2d(), 
    leftEncoder.getIntegratedSensorPosition()/(PIDConstants.CLICKS_PER_INCH*39.37), // 39.37 inches per meter
    rightEncoder.getIntegratedSensorPosition()/(PIDConstants.CLICKS_PER_INCH*39.37));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoder.setIntegratedSensorPosition(0, PIDConstants.kTimeoutMs);
    rightEncoder.setIntegratedSensorPosition(0, PIDConstants.kTimeoutMs);
	  System.out.println("All sensors are zeroed.\n");
  }

  public double getNavXAccumelatedVelocity(){
    return Math.hypot(ahrs.getVelocityX(), ahrs.getVelocityY());
    //return ahrs.getVelocityX();
  }

  public double getAcceleration(){
    return Math.hypot(ahrs.getWorldLinearAccelX(), ahrs.getWorldLinearAccelY())*accelerationDueToGrativy;
    //return ahrs.getWorldLinearAccelX();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftTalons[0].getSelectedSensorVelocity() /(PIDConstants.TALON_VELOCITY_PER_ROBOT_VELOCITY*39.37), //39.37 inches per meter
      rightTalons[0].getSelectedSensorVelocity() / (PIDConstants.TALON_VELOCITY_PER_ROBOT_VELOCITY*39.37)
    );
  }

  public Pose2d getPose(){
    SmartDashboard.putNumber("X Pose", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y Pose", odometry.getPoseMeters().getY());
    return odometry.getPoseMeters();
  }

  public void zeroHeading(){
    ahrs.zeroYaw();
  }

  public Rotation2d getHeading(){
    SmartDashboard.putNumber("Heading", Math.IEEEremainder(ahrs.getAngle(),360));
    return Rotation2d.fromDegrees(Math.IEEEremainder(ahrs.getAngle(),360));
  }

  public double getTurnRate(){
    return -ahrs.getRate();
  }

  public void tankDrive(double forward, double turn){
    diffDrive.arcadeDrive(forward, turn);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SmartDashboard.putNumber("Left voltage ", leftVolts);
    SmartDashboard.putNumber("Right voltage ", rightVolts);
    leftTalons[0].setVoltage(leftVolts);
    rightTalons[0].setVoltage(-rightVolts);
    diffDrive.feed();
  }
  
  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    diffDrive.setMaxOutput(maxOutput);
  }
  

  /** 
   * Sets the motors' inversion for the entire drivetrain
   * 
   * @param leftInverted what the entire left side is set to
   * @param rightInverted what the entire right side is set to
  */
  public void setDrivetrainInverted(boolean leftInverted, boolean rightInverted){
    for(int i=0; i<PortConstants.LEFT_MOTORS_IDS.length; i++){
      leftTalons[i].setInverted(leftInverted);
    }

    for(int i=0; i<PortConstants.RIGHT_MOTORS_IDS.length; i++){
      rightTalons[i].setInverted(rightInverted);
    }
  }

  DifferentialDriveVoltageConstraint autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        TrajectoryConstants.kS,
        TrajectoryConstants.kV,
        TrajectoryConstants.kA),
        kinematics,
        10 //max voltage the trajectory will allow
      );

  TrajectoryConfig config =
    new TrajectoryConfig(
      TrajectoryConstants.maxVelocityMetersPerSecond,
      TrajectoryConstants.maxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(kinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

  public DifferentialDriveVoltageConstraint getDriveVoltageConstraint(){
    return autoVoltageConstraint;
  }

  public TrajectoryConfig getTrajectoryConfig(){
    return config;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  //idk how to make this not static yet
  public static Command getRamseteCommandBase(Drivetrain drivetrain, Trajectory trajectory){
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        drivetrain::getPose,
        new RamseteController(TrajectoryConstants.kRamseteB, TrajectoryConstants.kRamseteZeta),
        new SimpleMotorFeedforward(TrajectoryConstants.kS,
                                   TrajectoryConstants.kV,
                                   TrajectoryConstants.kA),
        drivetrain.getKinematics(),
        drivetrain::getWheelSpeeds,
        new PIDController(TrajectoryConstants.kP, 0, TrajectoryConstants.kD),
        new PIDController(TrajectoryConstants.kP, 0, TrajectoryConstants.kD),
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts,
        drivetrain
    );

    drivetrain.resetOdometry(trajectory.getInitialPose());
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

}
