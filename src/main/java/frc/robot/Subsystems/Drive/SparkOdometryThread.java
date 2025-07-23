package frc.robot.Subsystems.Drive;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Util.SparkUtil;
import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

public class SparkOdometryThread {
  private final ArrayList<SparkBase> sparks = new ArrayList<>();
  private final ArrayList<DoubleSupplier> sparkSignals = new ArrayList<>();
  private final ArrayList<DoubleSupplier> genericSignals = new ArrayList<>();
  private final ArrayList<Queue<Double>> sparkQueue = new ArrayList<>();
  private final ArrayList<Queue<Double>> genericQueue = new ArrayList<>();
  private final ArrayList<Queue<Double>> timeQueue = new ArrayList<>();

  private static SparkOdometryThread instance = null;
  private Notifier notifier = new Notifier(this::run);

  public static SparkOdometryThread getInstance() {
    if (instance != null) {
      return instance;
    } else {
      instance = new SparkOdometryThread();
      return instance;
    }
  }

  private SparkOdometryThread() {
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (timeQueue.size() > 0) {
      notifier.startPeriodic(1.0 / Constants.DriveConstants.odometryFrequency);
    }
  }

  public Queue<Double> registerSparkSignal(SparkBase spark, DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    // Drive.odometryLock.lock();
    try {
      sparks.add(spark);
      sparkSignals.add(signal);
      sparkQueue.add(queue);
    } finally {
      // Drive.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> registerGenericSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    // Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueue.add(queue);
    } finally {
      // Drive.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimeQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    // Drive.odometryLock.lock();
    try {
      timeQueue.add(queue);
    } finally {
      // Drive.odometryLock.unlock();
    }
    return queue;
  }

  private void run() {
    // Drive.odometryLock.lock();
    try {
      double timestamp = RobotController.getFPGATime() / 1e6;
      boolean good = true;
      double[] sparkValues = new double[sparkSignals.size()];
      for (int i = 0; i < sparkSignals.size(); i++) {
        sparkValues[i] = sparkSignals.get(i).getAsDouble();
        if (!SparkUtil.isOK(sparks.get(i))) {
          good = false;
        }
      }
      if (good) {
        for (int i = 0; i < sparkSignals.size(); i++) {
          sparkQueue.get(i).offer(sparkValues[i]);
          genericQueue.get(i).offer(genericSignals.get(i).getAsDouble());
          timeQueue.get(i).offer(timestamp);
        }
      }
    } finally {
      // Drive.odometryLock.unlock();
    }
  }
}
