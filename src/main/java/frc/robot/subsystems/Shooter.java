package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants;

public class Shooter extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Shooter mInstance;
  private PeriodicIO mPeriodicIO;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private TalonFX ShooterLeft;
  private TalonFX ShooterRight;
  private TalonFX ShooterPivot;
  private TalonFX ShooterExtend;
  private VelocityVoltage LeftShooterPID;
  private VelocityVoltage RightShooterPID;
  
  double LeftShooterEncoder = 0;
  double RightShooterEncoder = 0;

    private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private Shooter() {
    super("Shooter");

    mPeriodicIO = new PeriodicIO();

          /* Configure the Talon to use a supply limit of 1 amps IF we exceed 4 amps for over 1 second */
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.CurrentLimits.SupplyCurrentLimit = 1; // Limit to 1 amps
      config.CurrentLimits.SupplyCurrentThreshold = 4; // If we exceed 4 amps
      config.CurrentLimits.SupplyTimeThreshold = 1.0; // For at least 1 second
      config.CurrentLimits.SupplyCurrentLimitEnable = true; // And enable it
      config.CurrentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
      config.CurrentLimits.StatorCurrentLimitEnable = true; // And enable it
      ShooterLeft.getConfigurator().apply(config);
      ShooterRight.getConfigurator().apply(config);
    
    // <joe> when you create a new motor it is a good idea reset to factory defaults
    // <joe> Ramping here as well
    // <joe> See https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/CurrentLimits/src/main/java/frc/robot/Robot.java
    // <joe> for an example of current limits.  Ramping is fairly similar where you set the 
    // <joe> appropriate ramp period on the OpenLoopRampsConfigs or the ClosedLoopRampsConfigs
    ShooterLeft = new TalonFX(Constants.kShooterLeftMotorId);
    ShooterRight = new TalonFX(Constants.kShooterRightMotorId);
    ShooterPivot = new TalonFX(Constants.kIntakeTiltId);
    ShooterExtend = new TalonFX(Constants.kShooterExtensionMotorId);
    
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = Constants.kShooterP;
    slot0.kI = Constants.kShooterI;
    slot0.kD = Constants.kShooterD;
    slot0.kV = Constants.kShooterFF;
    ShooterLeft.getConfigurator ().apply(slot0);
    ShooterRight.getConfigurator ().apply(slot0);

    LeftShooterEncoder = ShooterLeft.getVelocity().refresh().getValue();
    RightShooterEncoder = ShooterRight.getVelocity().refresh().getValue();

    ShooterLeft.setNeutralMode(NeutralModeValue.Coast);
    ShooterRight.setNeutralMode(NeutralModeValue.Coast);
    ShooterPivot.setNeutralMode(NeutralModeValue.Brake);
    ShooterExtend.setNeutralMode(NeutralModeValue.Brake);

    ShooterLeft.setInverted(true);
    ShooterRight.setInverted(false);

  }

  private static class PeriodicIO {
    double shooter_rpm = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    double limitedSpeed = mSpeedLimiter.calculate(mPeriodicIO.shooter_rpm);
    ShooterLeft.setControl(LeftShooterPID.withVelocity(limitedSpeed));
    ShooterRight.setControl(RightShooterPID.withVelocity(limitedSpeed));
   
  }

  @Override
  public void stop() {
    stopShooter();
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed (RPM):", mPeriodicIO.shooter_rpm);
    putNumber("Left speed:", ShooterLeft.getVelocity().refresh().getValue());
    putNumber("Right speed:", ShooterRight.getVelocity().refresh().getValue());
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  // <joe> don't seem to have anything for the pivot motor or extension motor
  public void setSpeed(double rpm) {
    mPeriodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() {
    mPeriodicIO.shooter_rpm = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
