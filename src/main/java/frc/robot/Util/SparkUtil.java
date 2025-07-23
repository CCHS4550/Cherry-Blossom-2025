package frc.robot.Util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {
  // allows us to see if the sparkmax has had an error in the past
  public static boolean stickyFault = false;

  // only takes the sparks value if the spark isn't erroring
  public static void ifOk(
      SparkBase spark, DoubleSupplier doubleSupplier, DoubleConsumer doubleConsumer) {
    double num = doubleSupplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      doubleConsumer.accept(num);
    } else {
      stickyFault = true;
    }
  }

  // same thing but can take in values as a array
  public static void ifOK(
      SparkBase spark, DoubleSupplier[] doubleSuppliers, Consumer<double[]> doubleConsumers) {
    double[] nums = new double[doubleSuppliers.length];
    for (int i = 0; i < doubleSuppliers.length; i++) {
      nums[i] = doubleSuppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        stickyFault = true;
        return;
      }
    }
    doubleConsumers.accept(nums);
  }

  public static boolean isOK(SparkBase spark) {
    boolean isFine = true;
    if (spark.getLastError() != REVLibError.kOk) {
      isFine = false;
    }
    return isFine;
  }

  public static boolean isOK(SparkBase[] spark) {
    boolean isFine = true;
    for (int i = 0; i < spark.length; i++) {
      if (spark[i].getLastError() != REVLibError.kOk) {
        isFine = false;
        break;
      }
    }
    return isFine;
  }

  public static void makeItWork(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        stickyFault = true;
      }
    }
  }
}
