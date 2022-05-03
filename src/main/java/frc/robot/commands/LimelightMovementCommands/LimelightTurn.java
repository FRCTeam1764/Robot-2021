/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LimelightMovementCommands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.ShuffleboardInfo;
import frc.robot.commands.DriveCommands.JoystickDrive;
import frc.robot.commands.DriveCommands.XBoxDrive;
import frc.robot.constants.FieldDimensionConstants;
import frc.robot.constants.RobotDimensionConstants;
import frc.robot.util.Limelight;

public class LimelightTurn extends CommandBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  DifferentialDrive diffDrive;


  public double getDistance(){
    y = ty.getDouble(0.0);
    double a1 = RobotDimensionConstants.LIMELIGHT_CAMERA_ANGLE;
    double h1 = RobotDimensionConstants.LIMELIGHT_HEIGHT_FROM_GROUND;
    double h2 = FieldDimensionConstants.POWER_PORT_VISION_TRACKING_HEIGHT;
    double distance = (h2-h1)/Math.tan(Math.toRadians(y+a1));
    return distance;
  }

  //the minimum speed where, if the motor is going that speed, the robot will not move
  final double absouluteMinSpeed = .19;
  double kp;
  double angleTolerance;
  double minCommand = 0.035;
  double steeringAdjust;

  NetworkTableEntry kpEntry, angleToleranceEntry;
  public LimelightTurn() {
    kpEntry = ShuffleboardInfo.getInstance().getKpEntry();
    angleToleranceEntry = ShuffleboardInfo.getInstance().getAnlgeToleranceEntry();

    kp = kpEntry.getDouble(0);
    angleTolerance = angleToleranceEntry.getDouble(0);

    addRequirements(Robot.drivetrain);
    diffDrive = Robot.drivetrain.diffDrive;
  }

  public void turnRobot(){
    x = tx.getDouble(0.0);
    double minCommand = 0.035;
    double steeringAdjust = kp*x;
   
    if(Math.abs(x)>angleTolerance){
      if(Math.abs(steeringAdjust) <= absouluteMinSpeed){
        steeringAdjust = (minCommand + absouluteMinSpeed) * Math.signum(steeringAdjust);
      }
    }
    else{
      steeringAdjust = 0;
      diffDrive.arcadeDrive(0, 0);
    }

    //System.out.println(steeringAdjust + "," + x);
   // System.out.println("it reaches here");
    diffDrive.arcadeDrive(0, steeringAdjust);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.turnLEDOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("it also reaches here");
    turnRobot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("limelight ends");

    if(DriverStation.getInstance().getJoystickIsXbox(0)){
      CommandScheduler.getInstance().schedule(new XBoxDrive());
    }
    else{
      CommandScheduler.getInstance().schedule(new JoystickDrive());
    }
    //diffDrive.arcadeDrive(0, 0);
    //Robot.drivetrain.stopDrivetrain();
    //Limelight.turnLEDOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return Math.abs(x)<angleTolerance;
  }
}
