package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robotstate;
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

/**
 * this creates a drivetrain that tracks on field positions, and is able to move the entire bot
 * according to input uses a state machine function, so most operations should be able to be called
 * by simply changing the wanted state
 */
public class Drive extends SubsystemBase {

  // java lock to implement thread safe
  static final Lock odometryLock = new ReentrantLock();

  Transform2d tagTransform =
      new Transform2d(0.0, Units.inchesToMeters(20), Rotation2d.fromDegrees(150));
  Pose2d testPose =
      new Pose2d(
          Constants.VisionConstants.aprilTagLayout
              .getTagPose(20)
              .get()
              .toPose2d()
              .plus(tagTransform)
              .getTranslation(),
          Rotation2d.fromDegrees(60));
  Pose2d testPoseOG = Constants.VisionConstants.aprilTagLayout.getTagPose(20).get().toPose2d();

  // declare gyro
  private final GyroIO
      gyroIO; // the gyro interface used by drive, will be defined as gyroPigeon if real
  private final GyroIOInputsAutoLogged gyroInputs =
      new GyroIOInputsAutoLogged(); // the logged gyro inputs

  private final Module[] modules = new Module[4]; // the 4 modules

  private final SysIdRoutine sysId;

  // configure gyro disconnection alert
  private final Alert gyroDCAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // used to perform inverse kinematics to convert chassis speeds to individual module states
  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.DriveConstants.moduleTranslations);

  // variable used to track rotation of the robot
  private Rotation2d rawGyroRotation = new Rotation2d();

  // values to use during teleop, these will be periodically set during the default command
  private double xJoystickInput = 0.0;
  private double yJoystickInput = 0.0;
  private double omegaJoystickInput = 0.0;

  /**
   * Pid Controller for drive at angle.
   *
   * <p>creates a new pid controller with built in trapezoidal motion this is necesary because while
   * the pid controller built into the turn motor can handle turning a singular wheel to an angle, a
   * seperate pid must be called to give the overall angles that the bot is hitting
   */
  ProfiledPIDController angleController =
      new ProfiledPIDController(
          Constants.DriveConstants.ANGLE_KP,
          0.0,
          Constants.DriveConstants.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              Constants.DriveConstants.ANGLE_MAX_VELOCITY,
              Constants.DriveConstants.ANGLE_MAX_ACCELERATION));

  // potential bad practice
  // mainly used in path on the fly, nothing else uses command scheduler
  private boolean isRunningCommand =
      false; // exists in order to prevent the periodic state machine from calling the same command
  // multiple times
  private BooleanSupplier shouldCancelEarly =
      () ->
          false; // we can enable should cancel early anytime we want to stop a command from running
  // state we want drive train to be in

  public enum WantedState {
    SYS_ID,
    AUTO,
    TELEOP_DRIVE,
    IDLE
  }

  // state the drive train is in
  public enum SystemState {
    SYS_ID,
    AUTO,
    TELEOP_DRIVE,
    IDLE
  }

  // which sys id routine to run
  // note that sysID might also be used as a stand alone command, in which case we would have to
  // apply the isRunningCommand boolean
  // as of now not a concern as we will not be runnign sysID
  public enum SysIdtoRun {
    NONE,
    DRIVE_Wheel_Radius_Characterization,
    FEED_FORWARD_Calibration,
    QUASISTATIC_FORWARD,
    QUASISTATIC_REVERSE,
    DYNAMIC_FORWARD,
    DYNAMIC_REVERSE
  }

  // initialize our states
  private SystemState systemState = SystemState.TELEOP_DRIVE;
  private WantedState wantedState = WantedState.TELEOP_DRIVE;
  private SysIdtoRun sysIdtoRun = SysIdtoRun.NONE;

  // array of our previous module positions
  private SwerveModulePosition[] lastModulePos =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  // creates a swervedrive pose estimator, can be used to fuse with vision and does the math needed
  // to update given our bots information
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePos, new Pose2d());

  /**
   * constructor for drive train
   *
   * @param gyroIO instance of gyroIO or classes that implement gyroIO
   * @param flModuleIO instance of moduleIO or classes that implement moduleIO
   * @param frModuleIO instance of moduleIO or classes that implement moduleIO
   * @param blModuleIO instance of moduleIO or classes that implement moduleIO
   * @param brModuleIO instance of moduleIO or classes that implement moduleIO
   */
  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    // initialize the gyro and modules and sim
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // initiate robot state with null values
    // TODO: maybe do this better
    Robotstate.getInstance().updateBotPoseAndSpeeds(new Pose2d(), new ChassisSpeeds());
    Robotstate.getInstance().updateRawGyroVelo(0.0);

    // Usage reporting
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // begin the odometry thread
    SparkOdometryThread.getInstance().start();

    // create our autobuilder for pathfinder
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(1.6, 0.0, 0.3), new PIDConstants(5.0, 0.0, 0.0)),
        Constants.DriveConstants.ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    // make sure our angle controller wraps angles properly

    // use our logged AD* algorithm as the pathfinder
    Pathfinding.setPathfinder(new LocalADStarAK());

    // logging
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // create the sysID routine
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    Logger.recordOutput("testing/adjust", testPose);
    Logger.recordOutput("testing/og", testPoseOG);
  }

  @Override
  public void periodic() {
    // lock the thread for thread safe
    odometryLock.lock();
    try {
      // update and log gyro inputs
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);

      // run modules periodic method
      for (var module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    // stop if disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      halt();
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // odometry calcs
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // get an array of timestamps from modules
    int sampleCount = sampleTimestamps.length; // number of samples to work through

    // loop for all samples of odometry information
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions =
          new SwerveModulePosition[4]; // position of each module
      SwerveModulePosition[] moduleDeltas =
          new SwerveModulePosition[4]; // change in position of each module
      // loop through all modules in a given sample timestamp
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] =
            modules[moduleIndex].getOdometryPositions()[i]; // set the position at given timestamp

        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePos[moduleIndex].distanceMeters,
                modulePositions[moduleIndex]
                    .angle); // find difference in the module position of given timestamp vs
        // previous
        lastModulePos[moduleIndex] =
            modulePositions[moduleIndex]; // set previous timestamp module position
      }

      // update gyro information
      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i]; // update using our real gyro
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation =
            rawGyroRotation.plus(new Rotation2d(twist.dtheta)); // update using robot odometry
      }

      // alert of gyro is disconnected
      gyroDCAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

      poseEstimator.updateWithTime(
          sampleTimestamps[i],
          rawGyroRotation,
          modulePositions); // use poseEstimators inbuild function to update with our newest
      // odometry information, coordinated with timestamps
    }

    systemState = handleStateTransition(); // adjust system state according to the wanted state

    // log states
    Logger.recordOutput("Subsystems/Drive/SystemState", systemState);
    Logger.recordOutput("Subsystems/Drive/DesiredState", wantedState);

    // turn the states into desired output
    applyStates();

    // for (int i = 0; i < 3; i++) {
    //   modules[i].runCharacterization(10);
    // }

    Robotstate.getInstance().updateBotPoseAndSpeeds(getPose(), getChassisSpeeds());
    Robotstate.getInstance().updateRawGyroVelo(gyroInputs.yawVelocityRadPerSec);
  }

  /**
   * sets the system state to be the same as the wanted state, but can be set to perform more
   * complex judgements on what state to goto if so desired
   *
   * @return the systemstate that our systemState variable will be set to
   */
  private SystemState handleStateTransition() {

    return switch (wantedState) {
      case SYS_ID -> SystemState.SYS_ID;
      case AUTO -> SystemState.AUTO;
      case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
      default -> SystemState.IDLE;
    };
  }

  // perform a desired outcome depending on our state
  private void applyStates() {
    switch (systemState) {
      default:
      case SYS_ID:
        runSysID();
        break;
      case AUTO: // AUTO just automatically breaks because it is made of a completely planned set of
        // commands to follow that don't require a periodic state system
        break;
      case TELEOP_DRIVE:
        Logger.recordOutput("Subsystems/Drive/ joystick x", xJoystickInput);
        Logger.recordOutput("Subsystems/Drive/ joystick y", yJoystickInput);
        Logger.recordOutput("Subsystems/Drive/ joystick omega", omegaJoystickInput);

        joystickDrive(xJoystickInput, yJoystickInput, omegaJoystickInput);
        // calculates speeds
        // runs velocity
        break;
      case IDLE:
        break;
    }
  }

  /**
   * turns a set of overall robot relative speeds into individual module instructions for the motors
   * to then follow
   *
   * @param speeds the desired chassis speeds, in XY units of meters per second and omega units of
   *     radians per second
   */
  public void runVelocity(ChassisSpeeds speeds) {

    // calculate and optomize our given speeds
    ChassisSpeeds discreteSpeeds =
        ChassisSpeeds.discretize(
            speeds,
            0.02); // seperate individual velocity components for a given timestamp. Keep in mind
    // this can be thrown off if the speeds are later scaled
    SwerveModuleState[] setPointStates =
        kinematics.toSwerveModuleStates(
            discreteSpeeds); // turn the speeds to module states(drive motor speed and turn motor
    // angle)
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setPointStates,
        Constants.DriveConstants
            .maxSpeedMetersPerSec); // normalizes wheel velocity if any individual modules are above
    // the max speed. Keep in mind that if this is called, the
    // discretization will be innaccurate

    // logging
    Logger.recordOutput("SwerveStates/Setpoints", setPointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // set all modules to our found states. Note that we still have to optomize our wheel angle for
    // better wraparound
    // set to 3 to ignore the broken swerve module? fix once fixed
    for (int i = 0; i < 4; i++) {
      modules[i].runSwerveState(setPointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setPointStates);
  }

  /**
   * turns a set of overall speeds into individual module instructions for the motors to then follow
   * but with a max turning velo
   *
   * @param speeds the desired chassis speeds, in XY units of meters per second and omega units of
   *     radians per second
   * @param maxTurnVelocityRadiansPerSecond the max turn velo, in radians per second. It will not
   *     normalize around this number but rather act as a hard cap
   */
  public void runVelocityWithMaxTurnVelo(
      ChassisSpeeds speeds, double maxTurnVelocityRadiansPerSecond) {

    // limits our requested rotational speed to a maxium velocity before desscretizing to avoid any
    // unintentionalskew
    // simply setting a max cap and not a scale because I dont care how it gets up to this max velo
    // keep in mind that this is potential bad practice or a misinterpretation of the following
    // methods
    if (speeds.omegaRadiansPerSecond > maxTurnVelocityRadiansPerSecond) {
      speeds.omegaRadiansPerSecond = maxTurnVelocityRadiansPerSecond;
    }

    // calculate and optomize our given speeds
    ChassisSpeeds discreteSpeeds =
        ChassisSpeeds.discretize(
            speeds,
            0.02); // seperate individual velocity components for a given timestamp. Keep in mind
    // this can be thrown off if the speeds are later scaled
    SwerveModuleState[] setPointStates =
        kinematics.toSwerveModuleStates(
            discreteSpeeds); // turn the speeds to module states(drive motor speed and turn motor
    // angle)
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setPointStates,
        Constants.DriveConstants
            .maxSpeedMetersPerSec); // normalizes wheeel velocity if any individual modules are
    // above the max speed. Keep in mind that if this is called, the
    // discretization will be innaccurate

    // logging
    Logger.recordOutput("SwerveStates/Setpoints", setPointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // set all modules to our found states. Note that we still have to optomize our wheel angle for
    // better wraparound
    for (int i = 0; i < 4; i++) {
      modules[i].runSwerveState(setPointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setPointStates);
  }
  /**
   * This method takes in our various joystick inputs, and converts them into a chassis speed while
   * applying deadband then, the bot is set to run at the speeds everything should be field relative
   *
   * @param xInput the horizontal joystick input
   * @param yInput the vertical joystick input
   * @param omegaInput rotatational joystick input
   */
  public void joystickDrive(double xInput, double yInput, double omegaInput) {

    // convert the 2 seperate x & y inputs into an overall translation 2d of 1 linear speed, just
    // found as the hypotenuse of the x & y
    // Translation2d linearVelocity =
    // getLinearVelocityFromXY(xInput, yInput, Constants.DriveConstants.deadband);

    // Apply rotation deadband
    double omega = MathUtil.applyDeadband(omegaInput, Constants.DriveConstants.deadband);
    // double omega = 0;

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    // Convert to field relative speeds
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xInput * getMaxLinearSpeed(),
            yInput * getMaxLinearSpeed(),
            omega * getMaxAngularSpeed());
    // boolean isFlipped =
    // DriverStation.getAlliance().isPresent()
    //     && DriverStation.getAlliance().get() == Alliance.Red;

    // set the bot to run at the chassis speeds
    runVelocity(
        // ChassisSpeeds.fromFieldRelativeSpeeds(
        speeds
        // , isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation())
        );
  }

  // this exists just to prevent too many states or apply states from being crowded
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
        sysIdDynamic(SysIdRoutine.Direction.kReverse);
        break;
    }
  }
  /**
   * converts a set of x and y speeds into a singular linear velocity - should be field oriented
   *
   * @param x the horizontal speed, units are arbitrary but should be meters per second for
   *     consistency
   * @param y the vertical speed, units are arbitrary but should be meters per second for
   *     consistency
   * @return a translation 2d of the linear velocity, with the direction found through trig
   */
  private static Translation2d getLinearVelocityFromXY(double x, double y) {

    // calculate the speed
    double linearMagnitude = Math.hypot(x, y);
    // calculate the direction
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
  /**
   * same thing as the other getLinearVeloFromXY method but applies deadband for teleop converts a
   * set of x and y speeds into a singular linear velocity - should be field oriented
   *
   * @param x the horizontal speed, units are arbitrary but should be meters per second for
   *     consistency
   * @param y the vertical speed, units are arbitrary but should be meters per second for
   *     consistency
   * @return a translation 2d of the linear velocity, with the direction found through trig
   */
  private static Translation2d getLinearVelocityFromXY(double x, double y, double deadband) {
    // Apply deadband and calculate speed
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);

    // find direction
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Runs the drive in a straight line with the specified drive output.
   *
   * @param output the desired output
   */
  public void runCharacterization(Double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** stops the bot */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * @return a null command to clear the command scheduler and halt any running commands
   */
  public Command halt() {
    return Commands.runOnce(() -> {}, this);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void StopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] =
          Constants.DriveConstants.moduleTranslations[i].getAngle(); // sets the module angles to x
    }
    kinematics.resetHeadings(headings); // applies the headings
    stop();
  }

  /**
   * In this test, the mechanism is gradually sped-up such that the voltage corresponding to
   * acceleration is negligible (hence, "as if static")
   *
   * @param direction whether to test this forward or reverse
   * @return a command to perform the desired sysID test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }
  /**
   * In this test, a constant ‘step voltage’ is given to the mechanism, so that the behavior while
   * accelerating can be determined.
   *
   * @param direction whether to perform this forward or reverse
   * @return a command to perform the desired sysID test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * log and get our current swerve module states
   *
   * @return an array of all 4 swerve module states
   */
  @AutoLogOutput(key = "SwerveStates/measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * getter for our current swerve module position very similar to module states, but state measures
   * drive velocity, while positions measures distance moved
   *
   * @return an array of all 4 swerve module positon
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 3; i++) { // only return 3 modules because I think one failed
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /**
   * log and getter for our bots current robot relative speed and angle, given as a Chassisspeeds
   *
   * @return a chassisSpeeds containing our bots speed and direction
   */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates()); // uses inverse kinematics
  }

  /**
   * log and getter for the position of our drive wheel when performing our wheel characterization
   * said test is not yet written
   *
   * @return an array of drive motor information for characterizing wheel radius
   */
  public double[] getWheelRadiusCharacterizationPosition() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * log and getter for module velo when doing a ff test for the drive motor said test is not yet
   * written
   *
   * @return an overall velo that combines all of the modules velo
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /**
   * @return the pose of the bot
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @return the rotation of the bot
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * @return the rotational speed of the bot in radians per second
   */
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /**
   * set the bot to a pose
   *
   * @param pose the pose to set the bot to
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    Pose2d robotPose = new Pose2d(pose.getTranslation(), rawGyroRotation);
    Robotstate.getInstance().setPose(robotPose);
  }

  /**
   * @return the max XY speed the bot can travel at in meters per sec
   */
  public double getMaxLinearSpeed() {
    return Constants.DriveConstants.maxSpeedMetersPerSec;
  }

  /**
   * @return the max omega speed the bot can travel at in radians per sec
   */
  public double getMaxAngularSpeed() {
    return Constants.DriveConstants.maxSpeedMetersPerSec / Constants.DriveConstants.driveBaseRadius;
  }

  /**
   * sets the bot's wanted state should be the primary way of manipulating the drivetrain outside of
   * the class
   *
   * @param wantedState the desired state
   */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  /**
   * sets the joystick x, this will be called in the default command
   *
   * @param x horizontal joystick input
   */
  public void setXJoystickInput(double x) {
    xJoystickInput = x;
  }

  /**
   * sets the joystick y, this will be called in the default command
   *
   * @param y vertical joystick input
   */
  public void setYJoystickInput(double y) {
    yJoystickInput = y;
  }

  /**
   * sets the joystick omega, this will be called in the default command
   *
   * @param omega the omega joystick input
   */
  public void setOmegaJoystickInput(double omega) {
    omegaJoystickInput = omega;
  }
}
