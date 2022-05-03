package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TimedMovementGroup;
import frc.robot.commands.DriveCommands.FarmingSimulatorDrive;
import frc.robot.commands.DriveCommands.JoystickDrive;
import frc.robot.commands.DriveCommands.XBoxDrive;
import frc.robot.commands.LimelightMovementCommands.LimelightTurn;
import frc.robot.commands.PIDMovementCommands.PIDDrive;
import frc.robot.commands.PIDMovementCommands.PIDDrive.PIDDriveControlType;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.AutoSubsystems.PIDMovement;
import frc.robot.constants.PortConstants;

public class Robot extends TimedRobot {

  public static Drivetrain drivetrain = new Drivetrain();

  public static int ballCount = 0;
  public static OI oi = new OI();
  public static SharpIRSensor feederIRSensor = new SharpIRSensor(PortConstants.SHARP_IR_SENSOR_FEEDER_ANALOG_PORT);
  public static SharpIRSensor intakeIRSensor = new SharpIRSensor(PortConstants.SHARP_IR_SENSOR_INTAKE_ANALOG_PORT);
  public static DoubleSolenoid climberSolenoid = new DoubleSolenoid(PortConstants.LEFT_CLIMBER_SOLENOID_PORT, PortConstants.RIGHT_CLIMBER_SOLENOID_PORT);

  public static DoubleSolenoid controlPanelWheelExtender = new DoubleSolenoid(PortConstants.CONTROL_PANEL_FORWARD_PORT, PortConstants.CONTROL_PANEL_REVERSE_PORT);

  JoystickDrive joystickdrive = new JoystickDrive();
  XBoxDrive xboxdrive = new XBoxDrive();
  FarmingSimulatorDrive jaxonDumbDrive = new FarmingSimulatorDrive();

  private ShuffleboardInfo sbiInstance = ShuffleboardInfo.getInstance();

  @Override
  public void robotInit() {
    //ShuffleboardCamera.camera = CameraServer.getInstance().startAutomaticCapture(ShuffleboardCamera.cameraPort);
   // ShuffleboardCamera.camera.setConnectVerbose(0);
  }
  
  @Override
  public void autonomousInit() {
    System.out.println("Start auto");
    CommandScheduler.getInstance().cancelAll();
    drivetrain.setDrivetrainNeturalMode(NeutralMode.Brake);
    CommandScheduler.getInstance().schedule(Drivetrain.getRamseteCommandBase(drivetrain, testTrajectory));
    //PIDMovement.setDistancePIDConfig(drivetrain.leftTalons[0], drivetrain.rightTalons[0]);
    //CommandScheduler.getInstance().schedule(new TimedMovementGroup());
   //CommandScheduler.getInstance().schedule(new PIDDrive(24, PIDDriveControlType.STRAIGHT));
    
    /*distance = 12;
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    drivetrain.leftTalons[0].configAllSettings(config);
    drivetrain.leftTalons[0].setSelectedSensorPosition(0);*/
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    drivetrain.setDrivetrainNeturalMode(NeutralMode.Brake);
    climberSolenoid.set(Value.kReverse);
    drivetrain.setDrivetrainInverted(false, false);

    if(DriverStation.getInstance().getJoystickIsXbox(0)){
      CommandScheduler.getInstance().schedule(xboxdrive);
    }
    else{
      CommandScheduler.getInstance().schedule( joystickdrive);
    }
   
    }

	/**
	 * This function is called periodically during operator control
	 */
 // LimelightTurn distanceToGoal = new LimelightTurn();


	public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Feeder IR", feederIRSensor.getVoltage());
    //SmartDashboard.putNumber("IR Sensor Voltage", intakeIRSensor.getVoltage());
    
  }

  Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          new Translation2d(1, .5),
          new Translation2d(2, -.5)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      drivetrain.getTrajectoryConfig()
  );

    @Override
    public void testInit() {
        
      //CommandScheduler.getInstance().schedule(jaxonDumbDrive);
    }
  
    @Override
  public void testPeriodic() {

      CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    drivetrain.setDrivetrainNeturalMode(NeutralMode.Coast);
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
    
    super.disabledPeriodic();
  }
	
	
	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}
}
