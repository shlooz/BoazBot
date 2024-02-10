package frc.robot;

import com.revrobotics.SparkPIDController;

public class Util {
  public static void updateSparkMaxPID(SparkPIDController sparkMax, double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput){
        sparkMax.setP(kP);
        sparkMax.setD(kD);
        sparkMax.setI(kI);
        sparkMax.setIZone(kIz);
        sparkMax.setFF(kFF);
        sparkMax.setOutputRange(kMinOutput, kMaxOutput);
}
}
