// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax leftShooterMotor;
  private final CANSparkMax rightShooterMotor;

  public ShooterSubsystem() {
    leftShooterMotor = new CANSparkMax(leftShooterMotorID, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(rightShooterMotorID, MotorType.kBrushless);

    leftShooterMotor.setIdleMode(IdleMode.kBrake);
    rightShooterMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShootingSpeed(double speed){
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
    
  }
}
