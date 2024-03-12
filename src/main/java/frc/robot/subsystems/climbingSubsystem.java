// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbingConstants.*;

import java.util.function.DoubleSupplier;

public class ClimbingSubsystem extends SubsystemBase {
  /** Creates a new climbingSubsystem. */
  static CANSparkMax leftClimbingMotor;
  static CANSparkMax rightClimbingMotor;

  public ClimbingSubsystem() {
    leftClimbingMotor = new CANSparkMax(LEFT_CLIMBING_MOTOR_ID, MotorType.kBrushless);
    rightClimbingMotor = new CANSparkMax(RIGHT_CLIMBING_MOTOR_ID, MotorType.kBrushless);

    leftClimbingMotor.restoreFactoryDefaults();
    rightClimbingMotor.restoreFactoryDefaults();

    leftClimbingMotor.setIdleMode(IdleMode.kBrake);
    rightClimbingMotor.setIdleMode(IdleMode.kBrake);

    leftClimbingMotor.setInverted(false);
    rightClimbingMotor.setInverted(true);

    leftClimbingMotor.setSmartCurrentLimit(LEFT_CLIMBING_MOTOR_ID);
    rightClimbingMotor.setSmartCurrentLimit(LEFT_CLIMBING_MOTOR_ID);

  }

  public static void enableCoast(boolean enable) {
    if (enable) {
      leftClimbingMotor.setIdleMode(IdleMode.kCoast);
      rightClimbingMotor.setIdleMode(IdleMode.kCoast);
    } else {
      leftClimbingMotor.setIdleMode(IdleMode.kBrake);
      rightClimbingMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbing(double leftSpeed, double rightSpeed){
    leftClimbingMotor.set(leftSpeed * CLIMBING_CONVERSION_FACTOR);
    rightClimbingMotor.set(rightSpeed * CLIMBING_CONVERSION_FACTOR);
  }

  public Command climbingCommand(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed){
    return new RunCommand(() -> climbing(leftSpeed.getAsDouble(), rightSpeed.getAsDouble()), this);
  }
}
