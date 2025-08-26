package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Robotstate {
  private static Robotstate instance;

  public static synchronized Robotstate getInstance() {
    if (instance == null) {
      instance = new Robotstate();
    }
    return instance;
  }

  public Pose2d pose;
  public ChassisSpeeds chassisSpeeds;
  public double gyroYawVelo;

  public synchronized void updateBotPose(Pose2d pose) {
    this.pose = pose;
  }
}
