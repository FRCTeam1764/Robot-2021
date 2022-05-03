// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class ShuffleboardInfo {
  private ShuffleboardTab shooterTab;

  private NetworkTableEntry distanceFromGoal, shooterSpeed;

  private static ShuffleboardInfo instance = null;

  private ShuffleboardInfo(){
    shooterTab = Shuffleboard.getTab("Shooter Debug");

    distanceFromGoal = shooterTab.add("distance from goal", 0).getEntry();
    shooterSpeed = shooterTab.add("shooter speed", 0).getEntry();
  }

  public static ShuffleboardInfo getInstance(){
    if( instance == null ){
        instance = new ShuffleboardInfo();
    }

    return instance;
    
}


  public NetworkTableEntry getShooterSpeedEntry(){
    return shooterSpeed;
  }

  public NetworkTableEntry getDistanceFromGoalEntry(){
    return distanceFromGoal;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class ShuffleboardInfo {
    private ShuffleboardTab limelightTab;
    private ShuffleboardTab shooterTab;
    private final NetworkTableEntry kp, angleTolerance;
    private final NetworkTableEntry motorSpeed;

    private static ShuffleboardInfo instance = null;
    
    
    private ShuffleboardInfo(){
        limelightTab = Shuffleboard.getTab("Limelight Debug");
        shooterTab = Shuffleboard.getTab("Shooter");
        kp = limelightTab.add("Steering KP", 0.02).getEntry();
        angleTolerance = limelightTab.add("Acceptable Angle", 0.8).getEntry();
        motorSpeed = shooterTab.add("Motor Speed (RPM)", 0).getEntry();
    }

    public static ShuffleboardInfo getInstance(){
        if( instance == null ){
            instance = new ShuffleboardInfo();
        }

        return instance;
    }

    public NetworkTableEntry getKpEntry(){
        return kp;
    }

    public NetworkTableEntry getAnlgeToleranceEntry(){
        return angleTolerance;
    }

    public NetworkTableEntry getMotorSpeedEntry(){
        return motorSpeed;
    }
}
