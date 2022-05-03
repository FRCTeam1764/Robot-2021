// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class TrajectoryConstants {
    public final static double maxVelocityMetersPerSecond = 2.7;
    public final static double maxAccelerationMetersPerSecondSquared = 3; //Not measured, just a guess

    //from robot characterization
    public final static double kS = .562;
    public final static double kV = 0.0529;
    public final static double kA = 0.00684;
    public final static double kP = 1.72e-21;
    public final static double kD = 0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
