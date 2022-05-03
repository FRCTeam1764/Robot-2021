/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.TeleopSubsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.ShuffleboardInfo;
import frc.robot.commands.ShooterCommand.ShooterControlMode;
import frc.robot.constants.PIDConstants;
import frc.robot.constants.PortConstants;

public class Shooter extends SubsystemBase {
  
  public WPI_TalonFX shooterMaster;
  WPI_TalonFX shooterFollower;

  double shooterVelocity = 0;
  ShooterControlMode controlMode;
  public double timeDuration = -1;
  double shooter;

  public static boolean shooterOn = true;
  public static boolean upwardHood = false;

  public static final double shooterRPM = 3300.0;

  public Shooter(double shooterVelocity, ShooterControlMode controlMode) {
    this.shooterVelocity = shooterVelocity;
    this.controlMode = controlMode;

    shooterMaster = configShooterMotors(PortConstants.SHOOTER_MASTER_MOTOR_PORT, true, false);
    shooterFollower = configShooterMotors(PortConstants.SHOOTER_FOLLOWER_MOTOR_PORT, false, true);

    shooterFollower.follow(shooterMaster);
  }

  public Shooter(double shooterVelocity, ShooterControlMode controlMode, double timeDuration) {
    this.shooterVelocity = shooterVelocity;
    this.controlMode = controlMode;
    this.timeDuration = timeDuration;

    shooterMaster = configShooterMotors(PortConstants.SHOOTER_MASTER_MOTOR_PORT, true, false);
    shooterFollower = configShooterMotors(PortConstants.SHOOTER_FOLLOWER_MOTOR_PORT, false, true);

    shooterFollower.follow(shooterMaster);
  }

  double velocity;

  @Override
  public void periodic() {
    velocity = shooterMaster.getSelectedSensorVelocity(0);
    //SmartDashboard.putNumber("Shooter's Velocity", velocity);
    // SmartDashboard.putBoolean("Shooter Status", velocity>9500.0 );
     //SmartDashboard.putBoolean("Shooter Toggle", shooterOn);
    
  }

  //for a constant shooter velocity (in rpm)
  public void shoot() {
    if(controlMode == ShooterControlMode.PID){
      SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
      double kF = simpleMotorFeedforward.calculate(shooterVelocity);
      shooterMaster.set(ControlMode.Velocity, shooterVelocity/60*2048*0.1);
    }
    else{
      shooterMaster.set(ControlMode.Velocity, 
      ShuffleboardInfo.getInstance().getMotorSpeedEntry().getDouble(0.0)/60*2048*0.1); //converts it from rpm
    }

  }

  //for a non-constant shooter veloctiy (in rpm)
  public void shoot(double velocity) {
    if(controlMode == ShooterControlMode.PID){
      SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
      double kF = simpleMotorFeedforward.calculate(velocity);
      shooterMaster.set(ControlMode.Velocity, velocity/60*2048*0.1);
    }
    else{
      shooterMaster.set(ControlMode.PercentOutput, velocity);
    }

  }

  public void shoot(float time){
    Timer timer = new Timer();
    timer.start();

    while(timer.get()<=time){
      shoot();
    }

    timer.stop();
    stopShooter();
  }

  public void stopShooter(){
    shooterMaster.set(ControlMode.PercentOutput,0);
  
  }

  //the distance from the power port where the hood should extend or retract
  public final double changeHoodDistance = 1; 

  public void recallibrateHood(double distance){
    Value extend = distance<=changeHoodDistance ? Value.kForward : Value.kReverse;
    upwardHood = extend.equals(Value.kForward);
    Robot.climberSolenoid.set(extend);
  }

  //From measured data, it calculates the velocity needed to at least hit the outer goal
  public static double calculateVelocity(double Distance){
    return 0;
  }

  public static WPI_TalonFX configShooterMotors(int portNum, boolean isMaster, boolean isInverted) {
    WPI_TalonFX talon = Robot.drivetrain.configTalons(portNum, isMaster, isInverted);
    talon.setNeutralMode(NeutralMode.Coast);

    if(isMaster){
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
      talon.configAllSettings(config);

      talon.config_kP(PIDConstants.kSlot_Shooter_Velocity, 0.0, PIDConstants.kTimeoutMs);
      talon.config_kI(PIDConstants.kSlot_Shooter_Velocity, 0.0, PIDConstants.kTimeoutMs);
      talon.config_kD(PIDConstants.kSlot_Shooter_Velocity, 0.0001, PIDConstants.kTimeoutMs);
      talon.config_kF(PIDConstants.kSlot_Shooter_Velocity, 0.049, PIDConstants.kTimeoutMs);
      talon.config_IntegralZone(PIDConstants.kSlot_Shooter_Velocity, (int)PIDConstants.kGains_Velocity_Shooter.kIzone, PIDConstants.kTimeoutMs);
      talon.configClosedLoopPeakOutput(PIDConstants.kSlot_Shooter_Velocity, PIDConstants.kGains_Velocity_Shooter.kPeakOutput, PIDConstants.kTimeoutMs);
      talon.configAllowableClosedloopError(PIDConstants.kSlot_Shooter_Velocity, 0, PIDConstants.kTimeoutMs);
      talon.selectProfileSlot(PIDConstants.kSlot_Shooter_Velocity, PIDConstants.PID_PRIMARY);
      //talon = setPIDShooterVelocityConfig(talon);
    }

    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0);

    return talon;
  }

  
}
