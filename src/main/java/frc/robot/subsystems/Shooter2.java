package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.subsystems.leds.LEDs;

public class Shooter extends Subsystem {
    private static final double s_pivotMotorP = 0.12;
    private static final double s_pivotMotorI = 0.0;
    private static final double s_pivotMotorD = 0.001;

    private final PIDController s_pivotPID = new PIDController(s_pivotMotorP, s_pivotMotorI, s_pivotMotorD);

    private final DutyCycleEncoder s_pivotEncoder = new DutyCycleEncoder(Constants.Intake.k_pivotEncoderId);
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
  private TalonFX sPivotMotor;
  private TalonFX sShooterExtensionMotor;
  private VelocityVoltage mLeftShooterPID;
  private VelocityVoltage mRightShooterPID;
double mLeftShooterEncoder = 0;
double mRightShooterEncoder = 0;

    private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private Shooter() {
    super("Shooter");

    mPeriodicIO = new PeriodicIO();

    mLeftShooterMotor = new TalonFX(Constants.kShooterLeftMotorId);
    mRightShooterMotor = new TalonFX(Constants.kShooterRightMotorId);
    sPivotMotor = new TalonFX(Constants.kPivotMotorId);
    sShooterExtensionMotor = new TalonFX(Constants.kShooterExtensionMotorId);
    
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = Constants.kShooterP;
    slot0.kI = Constants.kShooterI;
    slot0.kD = Constants.kShooterD;
    slot0.kV = Constants.kShooterFF;
    //mLeftShooterPID.setOutputRange(Constants.kShooterMinOutput, Constants.kShooterMaxOutput)//TODO Find outputrange equivelent for CTRE
    //mLeftShooterPID.configClosedLoopOutputRange(Constants.kShooterMinOutput, Constants.kShooterMaxOutput);//possible fix to above issue
    mLeftShooterMotor.getConfigurator ().apply(slot0);
    mRightShooterMotor.getConfigurator ().apply(slot0);
    
    mLeftShooterEncoder = mLeftShooterMotor.getVelocity().refresh().getValue();
    mRightShooterEncoder = mRightShooterMotor.getVelocity().refresh().getValue();

    mLeftShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    mRightShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    sPivotMotor.setNeutralMode(NeutralModeValue.Brake);
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
    //mLeftShooterPID.setReference(limitedSpeed, ControlType.kVelocity);//todo revisit
    //mLeftShooterPID.set(ControlMode.Velocity, limitedSpeed);// possible fix for above comment
    //mRightShooterPID.setReference(limitedSpeed, ControlType.kVelocity);//todo revisit
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

  public void setSpeed(double rpm) {
    mPeriodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() {
    mPeriodicIO.shooter_rpm = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}

public class Shooter2 {
    
}
