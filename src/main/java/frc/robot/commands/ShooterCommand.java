/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.ShuffleboardInfo;
import frc.robot.subsystems.TeleopSubsystems.Shooter;

public class ShooterCommand extends CommandBase {
  
  Shooter shooter;

  public enum ShooterControlMode{
    PID,
    STANDARD,
    TIMED
  }

  //for debugging
  NetworkTableEntry shooterVelocity, distanceFromGoal;

  public ShooterCommand(double shooterMotorSpeed, ShooterControlMode controlMode) {
    if(shooterMotorSpeed == -1){
      shooterMotorSpeed = ShuffleboardInfo.getInstance().getShooterSpeedEntry().getDouble(0.0);
    }
    shooter = new Shooter(shooterMotorSpeed, controlMode);

    addRequirements(shooter);
  }

  public ShooterCommand(double shooterMotorSpeed, ShooterControlMode controlMode, double timeDuration) {
    shooter = new Shooter(shooterMotorSpeed, controlMode, timeDuration);

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.shooterOn = !Shooter.shooterOn;

    //uncomment later
    //shooter.recallibrateHood(ShuffleboardInfo.getInstance().getDistanceFromGoalEntry().getDouble(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.timeDuration >0){
      shooter.shoot(shooter.timeDuration);
      end(false);
    }
    else{
      double distance = ShuffleboardInfo.getInstance().getDistanceFromGoalEntry().getDouble(0);
      shooter.shoot(/*Shooter.calculateVelocity(distance)*/);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    Shooter.shooterOn = !Shooter.shooterOn;
    Robot.ballCount = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
