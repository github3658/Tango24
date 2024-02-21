package frc.robot.controls;

public class SquaredInput implements InputScaler {
  private final double m_deadbandLimit;

  public SquaredInput(double deadbandLimit) {
    this.m_deadbandLimit = deadbandLimit;
  }

  public double scale(double input) {
    double numeratorFactor = (Math.abs(input) - m_deadbandLimit);
    double denominatorFactor = 1 - m_deadbandLimit;
    // <joe> we tend to cube our joystick inputs instead of squaring them.  Then we don't need this sign logic
    double sign = (input >= 0 ? 1 : -1);

    return sign * ((numeratorFactor * numeratorFactor) / (denominatorFactor * denominatorFactor));
  }
}
