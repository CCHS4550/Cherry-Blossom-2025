package frc.robot.Subsystems.Turret.Rotation;

import static frc.robot.Constants.MechanismConstants.RotationConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

/** this class exists exclusively to bugfix the logic of higher level code */
public class RotationIOTest implements RotationIO {
  private double appliedVoltage = 0.0;
  private double previousLocationRadians = 0.0;
  private double previousUnwrappedRadians = 0.0;
  private double currentLocationRadiansWrapped = 0.0;
  private double currentLocationRadiansUnwrapped = 0.0;
  private double locationThetaRadians = 0.0;
  private PIDController testingPid = new PIDController(rotationKp, rotationKi, rotationKd);

  public RotationIOTest() {
    testingPid.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void updateInputs(RotationIOInputs inputs) {

    inputs.rotationAppliedVolts = appliedVoltage;

    // shitty calcs to increase position and velocity based on voltages
    double increasedRadians = (appliedVoltage * 0.007);
    if (appliedVoltage < 0.05 && appliedVoltage > -0.05) {
      increasedRadians = 0.0;
    }

    // increase radians
    double unwrappedRadians = previousLocationRadians + increasedRadians;
    currentLocationRadiansUnwrapped = unwrappedRadians;
    currentLocationRadiansWrapped = wrapRadians(unwrappedRadians);

    // theta for rudimentary velo measurement
    locationThetaRadians = unwrappedRadians - previousLocationRadians;

    // set the rudimentary movement and velocity calc
    inputs.rotationPositionRad = wrapRadians(unwrappedRadians);
    inputs.rotationVelocityRadPerSec =
        locationThetaRadians
            / 0.02; // velocity blips on wrap i think, not sure if this is a problem yet

    // these values don't matter for the rudimentary testing that we are doing
    inputs.rotationConnected = true;
    inputs.rotationCurrentAmps = 0.0;

    // update the previous radians
    previousUnwrappedRadians = unwrappedRadians;
    previousLocationRadians = wrapRadians(unwrappedRadians);
  }

  @Override
  public void setRotationOpenLoop(double voltage) {
    appliedVoltage = voltage;
  }

  @Override
  public void setRotationPos(Rotation2d angle) {
    appliedVoltage =
        testingPid.calculate(currentLocationRadiansWrapped, wrapRadians(angle.getRadians()));
  }

  @Override
  public void setRotationPos(Rotation2d angle, double arbFF) {
    appliedVoltage =
        testingPid.calculate(currentLocationRadiansWrapped, wrapRadians(angle.getRadians()))
            + arbFF;
  }

  // wraps in between 0 and 2 PI radians
  private double wrapRadians(double radians) {
    return (radians % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
  }
}
