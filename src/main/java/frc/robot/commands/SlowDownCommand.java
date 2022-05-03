/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.DriveCommands.XBoxDrive;

public class SlowDownCommand extends CommandBase {
  /**
   * Creates a new SlowDownCommand.
   */
  public SlowDownCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  boolean lastStateSlowMo;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastStateSlowMo = XBoxDrive.slowMo;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    XBoxDrive.slowMo = !XBoxDrive.slowMo;
    SmartDashboard.putBoolean("Slow Mode Active",true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Slow Mode Active",false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lastStateSlowMo!=XBoxDrive.slowMo;
  }
}
