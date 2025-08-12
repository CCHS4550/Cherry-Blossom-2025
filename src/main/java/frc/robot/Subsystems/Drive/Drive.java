package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Subsystems.Drive.Gyro.GyroIO;
import frc.robot.Subsystems.Drive.Gyro.GyroIOInputsAutoLogged;
import frc.robot.Subsystems.Drive.Module.*;
import frc.robot.Subsystems.Drive.Module.Module;
import frc.robot.Util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final SysIdRoutine sysId;
  private final Alert gyroDCAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.DriveConstants.moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();

  private Pose2d pathOntheFlyPose;
  private PathConstraints pathConstraintsOnTheFly;
  private double maxTransSpeedMpsOnTheFly;
  private double maxTransAccelMpssqOnTheFly;
  private double maxRotSpeedRadPerSecOnTheFly;
  private double maxRotAccelRadPerSecSqOnTheFly;
  private double idealEndVeloOntheFly;

  public double xJoystickInput = 0.0;
  public double yJoystickInput = 0.0;
  public double omegaJoystickInput = 0.0;
  public double maxOptionalTurnVeloRadiansPerSec = Double.NaN;
  public double maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);
  public Rotation2d joystickDriveAtAngleAngle = Rotation2d.fromRadians(0.0);

  private final PIDController autoDriveToPointController = new PIDController(3.0, 0, 0.1);
  private final PIDController teleopDriveToPointController = new PIDController(3.6, 0, 0.1);
  private Pose2d driveToPointPose = new Pose2d();

  public static final double goToPoseTranslationError = Units.inchesToMeters(0.5);

  private boolean isRunningCommand = false;
  public BooleanSupplier shouldCancelEarly = () -> false;

  public enum WantedState {
    SYS_ID,
    AUTO,
    TELEOP_DRIVE,
    TELEOP_DRIVE_AT_ANGLE,
    PATH_ON_THE_FLY,
    DRIVE_TO_POINT,
    IDLE
  }

  public enum SystemState {
    SYS_ID,
    AUTO,
    TELEOP_DRIVE,
    TELEOP_DRIVE_AT_ANGLE,
    PATH_ON_THE_FLY,
    DRIVE_TO_POINT,
    IDLE
  }

  public enum SysIdtoRun {
    NONE,
    DRIVE_Wheel_Radius_Characterization,
    FEED_FORWARD_Calibration,
    QUASISTATIC_FORWARD,
    QUASISTATIC_REVERSE,
    DYNAMIC_FORWARD,
    DYNAMIC_REVERSE
  }

  private SystemState systemState = SystemState.TELEOP_DRIVE;
  private WantedState wantedState = WantedState.TELEOP_DRIVE;
  private SysIdtoRun sysIdtoRun = SysIdtoRun.NONE;

  private SwerveModulePosition[] lastModulePos =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePos, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(flModuleIO, 2);
    modules[3] = new Module(frModuleIO, 3);

    // Usage reporting
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    SparkOdometryThread.getInstance().start();

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        Constants.DriveConstants.ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePos[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePos[moduleIndex] = modulePositions[moduleIndex];
      }
      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    systemState = handleStateTransition();

    Logger.recordOutput("Subsystems/Drive/SystemState", systemState);
    Logger.recordOutput("Subsystems/Drive/DesiredState", wantedState);

    isRunningCommand = cancelIfNearAndReturnFalse();

    applyStates();

    gyroDCAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case SYS_ID -> SystemState.SYS_ID;
      case AUTO -> SystemState.AUTO;
      case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
      case TELEOP_DRIVE_AT_ANGLE -> SystemState.TELEOP_DRIVE_AT_ANGLE;
      case PATH_ON_THE_FLY -> {
        if (!isRunningCommand) {
          yield SystemState.PATH_ON_THE_FLY;
        } else {
          yield SystemState.TELEOP_DRIVE;
        }
      }
      case DRIVE_TO_POINT -> SystemState.DRIVE_TO_POINT;
      default -> SystemState.IDLE;
    };
  }

  private void applyStates() {
    switch (systemState) {
      default:
      case SYS_ID:
        runSysID();
        break;
      case AUTO:
        break;
      case TELEOP_DRIVE:
        joystickDrive(xJoystickInput, yJoystickInput, omegaJoystickInput);
        break;
      case TELEOP_DRIVE_AT_ANGLE:
        driveAtAngle(xJoystickInput, yJoystickInput, joystickDriveAtAngleAngle);
        break;
      case PATH_ON_THE_FLY:
        if (!isRunningCommand) {
          setPathConstraintsOnTheFly();
          isRunningCommand = true;
          AutoBuilder.pathfindToPose(
                  pathOntheFlyPose, pathConstraintsOnTheFly, idealEndVeloOntheFly)
              .until(shouldCancelEarly);
        }
        break;
      case DRIVE_TO_POINT:
        driveToPoint();
        break;
    }
  }

  public void runVelocity(ChassisSpeeds speeds) {

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setPointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setPointStates, Constants.DriveConstants.maxSpeedMetersPerSec);

    Logger.recordOutput("SwerveStates/Setpoints", setPointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSwerveState(setPointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setPointStates);
  }

  public void runVelocityWithMaxTurnVelo(
      ChassisSpeeds speeds, double maxTurnVelocityRadiansPerSecond) {

    // limits our requested rotational speed to a maxium velocity before desscretizing to avoid any
    // unintentionalskew
    // simply setting a max cap and not a scale because I dont care how it gets up to this max velo
    if (speeds.omegaRadiansPerSecond > maxTurnVelocityRadiansPerSecond) {
      speeds.omegaRadiansPerSecond = maxTurnVelocityRadiansPerSecond;
    }

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setPointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setPointStates, Constants.DriveConstants.maxSpeedMetersPerSec);

    Logger.recordOutput("SwerveStates/Setpoints", setPointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSwerveState(setPointStates[i]);
    }
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setPointStates);
  }

  public void joystickDrive(double xInput, double yInput, double omegaInput) {
    Translation2d linearVelocity =
        getLinearVelocityFromXY(xInput, yInput, Constants.DriveConstants.deadband);

    // Apply rotation deadband
    double omega = MathUtil.applyDeadband(omegaInput, Constants.DriveConstants.deadband);

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeed(),
            linearVelocity.getY() * getMaxLinearSpeed(),
            omega * getMaxAngularSpeed());
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
  }

  public void driveAtAngle(double xInput, double yInput, Rotation2d angle) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            Constants.DriveConstants.ANGLE_KP,
            0.0,
            Constants.DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                Constants.DriveConstants.ANGLE_MAX_VELOCITY,
                Constants.DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    Translation2d linearVelocity = getLinearVelocityFromXY(xInput, yInput);
    double omega = angleController.calculate(getRotation().getRadians(), angle.getRadians());
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeed(),
            linearVelocity.getY() * getMaxLinearSpeed(),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
  }

  public void driveAtAngle(double xInput, double yInput, Rotation2d angle, double maxTurnVelo) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            Constants.DriveConstants.ANGLE_KP,
            0.0,
            Constants.DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                Constants.DriveConstants.ANGLE_MAX_VELOCITY,
                Constants.DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    Translation2d linearVelocity = getLinearVelocityFromXY(xInput, yInput);
    double omega = angleController.calculate(getRotation().getRadians(), angle.getRadians());
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeed(),
            linearVelocity.getY() * getMaxLinearSpeed(),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    runVelocityWithMaxTurnVelo(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()),
        maxTurnVelo);
  }

  public void driveToPoint() {
    var translationToDesiredPoint =
        driveToPointPose.getTranslation().minus(getPose().getTranslation());
    var linearDistance = translationToDesiredPoint.getNorm();
    var frictionConstant = 0.0;
    if (linearDistance >= Units.inchesToMeters(0.5)) {
      frictionConstant =
          Constants.DriveConstants.driveToPointStaticFrictionConstant
              * Constants.DriveConstants.maxSpeedMetersPerSec;
    }
    var directionOfTravel = translationToDesiredPoint.getAngle();
    var velocityOutput = 0.0;
    if (DriverStation.isAutonomous()) {
      velocityOutput =
          Math.min(
              Math.abs(autoDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
              maxVelocityOutputForDriveToPoint);
    } else {
      velocityOutput =
          Math.min(
              Math.abs(teleopDriveToPointController.calculate(linearDistance, 0))
                  + frictionConstant,
              maxVelocityOutputForDriveToPoint);
    }
    var xComponent = velocityOutput * directionOfTravel.getCos();
    var yComponent = velocityOutput * directionOfTravel.getSin();

    Logger.recordOutput("Subsystems/Drive/DriveToPoint/xVelocitySetpoint", xComponent);
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/yVelocitySetpoint", yComponent);
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/velocityOutput", velocityOutput);
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/linearDistance", linearDistance);
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/directionOfTravel", directionOfTravel);
    Logger.recordOutput("Subsystems/Drive/DriveToPoint/desiredPoint", driveToPointPose);

    if (Double.isNaN(maxOptionalTurnVeloRadiansPerSec)) {
      driveAtAngle(xComponent, yComponent, driveToPointPose.getRotation());
    } else {
      driveAtAngle(
          xComponent, yComponent, driveToPointPose.getRotation(), maxOptionalTurnVeloRadiansPerSec);
    }
  }

  public boolean cancelIfNearAndReturnFalse() {
    if ((systemState == SystemState.PATH_ON_THE_FLY && !DriverStation.isAutonomous())) {
      var distance = pathOntheFlyPose.getTranslation().minus(getPose().getTranslation()).getNorm();

      Logger.recordOutput("Subsystems/Drive/PathOnFlyTeleOp/distanceFromEndpoint", distance);

      if (MathUtil.isNear(0.0, distance, goToPoseTranslationError)) {
        setWantedState(WantedState.TELEOP_DRIVE);
        return false;
      } else {
        return true;
      }
    } else if ((systemState == SystemState.DRIVE_TO_POINT && !DriverStation.isAutonomous())) {
      var distance = driveToPointPose.getTranslation().minus(getPose().getTranslation()).getNorm();

      Logger.recordOutput("Subsystems/Drive/DriveToPointTeleOp/distanceFromEndpoint", distance);

      if (MathUtil.isNear(0.0, distance, goToPoseTranslationError)) {
        setWantedState(WantedState.TELEOP_DRIVE);
        return false;
      } else {
        return true;
      }
    } else if ((systemState == SystemState.DRIVE_TO_POINT && DriverStation.isAutonomous())) {
      var distance = driveToPointPose.getTranslation().minus(getPose().getTranslation()).getNorm();

      Logger.recordOutput("Subsystems/Drive/DriveToPointAuto/distanceFromEndpoint", distance);

      if (MathUtil.isNear(0.0, distance, goToPoseTranslationError)) {
        setWantedState(WantedState.AUTO);
        return false;
      } else {
        return true;
      }
    } else {
      return true;
    }
  }

  public void runSysID() {
    switch (sysIdtoRun) {
      default:
        break;
      case NONE:
        break;
        // TODO: write a command for this
      case DRIVE_Wheel_Radius_Characterization:
        break;
        // TODO: write a command for this too
      case FEED_FORWARD_Calibration:
        break;
      case QUASISTATIC_FORWARD:
        sysIdQuasistatic(SysIdRoutine.Direction.kForward);
        break;
      case QUASISTATIC_REVERSE:
        sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
        break;
      case DYNAMIC_FORWARD:
        sysIdDynamic(SysIdRoutine.Direction.kForward);
        break;
      case DYNAMIC_REVERSE:
        sysIdDynamic(SysIdRoutine.Direction.kForward);
        break;
    }
  }

  private static Translation2d getLinearVelocityFromXY(double x, double y) {
    // Apply deadband
    double linearMagnitude = Math.hypot(x, y);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  private static Translation2d getLinearVelocityFromXY(double x, double y, double deadband) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public void runCharacterization(Double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void StopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = Constants.DriveConstants.moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  @AutoLogOutput(key = "SwerveStates/measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }
  // leave this blank for now because I dont want to deal with vision rn
  public void addVisionMeasurement() {}

  public double getMaxLinearSpeed() { // meters per sec
    return Constants.DriveConstants.maxSpeedMetersPerSec;
  }

  public double getMaxAngularSpeed() { // radians per sec
    return Constants.DriveConstants.maxSpeedMetersPerSec / Constants.DriveConstants.driveBaseRadius;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void setXJoystickInput(double x) {
    xJoystickInput = x;
  }

  public void setYJoystickInput(double y) {
    yJoystickInput = y;
  }

  public void setOmegaJoystickInput(double omega) {
    omegaJoystickInput = omega;
  }

  public void setMaxOptionalTurnVeloRadiansPerSec(double speed) {
    maxOptionalTurnVeloRadiansPerSec = speed;
  }

  public void setAngleLockAngle(Rotation2d radians) {
    joystickDriveAtAngleAngle = radians;
  }

  public void setPathOntheFlyPose(Pose2d pose) {
    pathOntheFlyPose = pose;
  }

  public void setMaxTransSpeedOnTheFly(double speed) {
    maxTransSpeedMpsOnTheFly = speed;
  }

  public void setMaxTransAccelOnTheFly(double speed) {
    maxTransAccelMpssqOnTheFly = speed;
  }

  public void setMaxRotSpeedOnTheFly(double speed) {
    maxRotSpeedRadPerSecOnTheFly = speed;
  }

  public void setMaxRotAccelOnTheFly(double speed) {
    maxRotAccelRadPerSecSqOnTheFly = speed;
  }

  public void setIdealEndVeloOntheFly(double speed) {
    idealEndVeloOntheFly = speed;
  }

  public void setPathConstraintsOnTheFly() {
    pathConstraintsOnTheFly =
        new PathConstraints(
            maxTransSpeedMpsOnTheFly,
            maxTransAccelMpssqOnTheFly,
            maxRotSpeedRadPerSecOnTheFly,
            maxRotAccelRadPerSecSqOnTheFly);
  }

  public void setEarlyCancel(boolean should) {
    shouldCancelEarly = () -> should;
  }
}
