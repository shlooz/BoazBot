// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax leftShooterMotor;
  private final CANSparkMax rightShooterMotor;

  public ShooterSubsystem() {
    leftShooterMotor = new CANSparkMax(LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();

    leftShooterMotor.setInverted(false);
    rightShooterMotor.setInverted(true);

    leftShooterMotor.setIdleMode(IdleMode.kCoast);
    rightShooterMotor.setIdleMode(IdleMode.kCoast);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShootingSpeed(double leftSpeed, double rightSpeed){
    leftShooterMotor.set(leftSpeed);
    rightShooterMotor.set(rightSpeed);
    
  }

  public Command shootingCommand(double leftSpeed, double rightSpeed){
    return new  RunCommand(() -> setShootingSpeed(leftSpeed, rightSpeed));
  }
  
  public Command shootingCommand(){
    return shootingCommand(0, 0);
  }
}
