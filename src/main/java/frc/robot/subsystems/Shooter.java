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

  private TalonFX mLeftShooterMotor;
  private TalonFX mRightShooterMotor;
  private TalonFX sShooterPivotMotor;
  private TalonFX sShooterExtensionMotor;
  private VelocityVoltage mLeftShooterPID;
  private VelocityVoltage mRightShooterPID;
  
  double mLeftShooterEncoder = 0;
  double mRightShooterEncoder = 0;

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
      mLeftShooterMotor.getConfigurator().apply(config);
      mRightShooterMotor.getConfigurator().apply(config);
    
    // <joe> when you create a new motor it is a good idea reset to factory defaults
    // <joe> Ramping here as well
    // <joe> See https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/CurrentLimits/src/main/java/frc/robot/Robot.java
    // <joe> for an example of current limits.  Ramping is fairly similar where you set the 
    // <joe> appropriate ramp period on the OpenLoopRampsConfigs or the ClosedLoopRampsConfigs
    mLeftShooterMotor = new TalonFX(Constants.kShooterLeftMotorId);
    mRightShooterMotor = new TalonFX(Constants.kShooterRightMotorId);
    sShooterPivotMotor = new TalonFX(Constants.kPivotMotorId);
    sShooterExtensionMotor = new TalonFX(Constants.kShooterExtensionMotorId);
    
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = Constants.kShooterP;
    slot0.kI = Constants.kShooterI;
    slot0.kD = Constants.kShooterD;
    slot0.kV = Constants.kShooterFF;
    mLeftShooterMotor.getConfigurator ().apply(slot0);
    mRightShooterMotor.getConfigurator ().apply(slot0);

    mLeftShooterEncoder = mLeftShooterMotor.getVelocity().refresh().getValue();
    mRightShooterEncoder = mRightShooterMotor.getVelocity().refresh().getValue();

    mLeftShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    mRightShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    sShooterPivotMotor.setNeutralMode(NeutralModeValue.Brake);
    sShooterExtensionMotor.setNeutralMode(NeutralModeValue.Brake);

    mLeftShooterMotor.setInverted(true);
    mRightShooterMotor.setInverted(false);

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
    mLeftShooterMotor.setControl(mLeftShooterPID.withVelocity(limitedSpeed));
    mRightShooterMotor.setControl(mRightShooterPID.withVelocity(limitedSpeed));
   
  }

  @Override
  public void stop() {
    stopShooter();
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed (RPM):", mPeriodicIO.shooter_rpm);
    putNumber("Left speed:", mLeftShooterMotor.getVelocity().refresh().getValue());
    putNumber("Right speed:", mRightShooterMotor.getVelocity().refresh().getValue());
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
