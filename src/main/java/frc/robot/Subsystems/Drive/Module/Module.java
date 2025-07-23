package frc.robot.Subsystems.Drive.Module;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDCAlert;
  private final Alert turnDCAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDCAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDCAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double posMeters = inputs.odometryDrivePos[i] * Constants.DriveConstants.wheelRadiusMeters;
      Rotation2d angle = inputs.odometryTurnPos[i];
      odometryPositions[i] = new SwerveModulePosition(posMeters, angle);
    }
    // update alerts
    driveDCAlert.set(!inputs.driveConnected);
    turnDCAlert.set(!inputs.turnConnected);
  }

  public void runSwerveState(SwerveModuleState state) {
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPos);

    io.setDrivevelo(state.speedMetersPerSecond / Constants.driveConstants.wheelRadius);
    io.setTurnPos(state.angle);
  }

  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPos(new Rotation2d());
  }

  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public Rotation2d getAngle() {
    return inputs.turnPos;
  }

  public double getPosMeters() {
    return inputs.drivePosRad * Constants.DriveConstants.wheelRadiusMeters;
  }

  public double getVelo() { // meters per second
    return inputs.driveSpeedVelo * Constants.DriveConstants.wheelRadiusMeters;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPosMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelo(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePosRad;
  }

  public double getFFCharacterizationVelocity() {
    return inputs.driveSpeedVelo;
  }
}
