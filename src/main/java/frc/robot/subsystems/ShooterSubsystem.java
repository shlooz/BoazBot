// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.*;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax leftShooterMotor;
  private final CANSparkMax rightShooterMotor;

  private final SparkPIDController leftShooterPIDController;
  private final SparkPIDController rightShooterPIDController;

  private final SparkAnalogSensor leftShooterMotorAnlog;
  private final SparkAnalogSensor rightShooterMotorAnlog;

  private final PIDController leftMotorPIDController;
  private final PIDController rightMotorPIDController;

  public ShooterSubsystem() {
    leftShooterMotor = new CANSparkMax(LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();

    leftShooterMotor.setInverted(false);
    rightShooterMotor.setInverted(true);

    leftShooterMotor.setIdleMode(IdleMode.kCoast);
    rightShooterMotor.setIdleMode(IdleMode.kCoast);

    leftShooterPIDController = leftShooterMotor.getPIDController();
    rightShooterPIDController = rightShooterMotor.getPIDController();

    leftShooterMotorAnlog = leftShooterMotor.getAnalog();
    rightShooterMotorAnlog = rightShooterMotor.getAnalog();

    leftMotorPIDController = new PIDController(0.000175, 0.000001, 0);
    rightMotorPIDController = new PIDController(0.000175, 0.000001, 0);

    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShootingSpeed(double leftSpeed, double rightSpeed){
    leftShooterMotor.set(leftSpeed);
    rightShooterMotor.set(rightSpeed);

    // System.out.println(leftSpeed);
    // System.out.println(rightSpeed);
    
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(
        leftShooterMotorAnlog.getPosition(),
        leftShooterMotorAnlog.getVelocity());
  }
  public Command shootingCommand(double leftSpeed, double rightSpeed){
    return new  RunCommand(() -> setShootingSpeed(leftSpeed, rightSpeed), this);
  }

  // public Command setShooterRPM(double leftRPM, double rightRMP){
  //   return new ProxyCommand(() -> new TrapezoidProfileCommand(
  //     new TrapezoidProfile(SHOOTER_CONSTRAINTS), 
  //     state, null, null, null)
  //   );
  // }

  // public controllShootingMotor();
  
  public Command shootingCommand(){
    return shootingCommand(0, 0);
  }

  public void shootingRPM(double leftRPM, double rightRPM){
    // leftShooterPIDController.setReference(leftRPM, ControlType.kVelocity);
    // rightShooterPIDController.setReference(rightRPM, ControlType.kVelocity);

    rightShooterMotor.set(
      rightMotorPIDController.calculate(
        rightShooterMotorAnlog.getVelocity(), rightRPM
      ));
    leftShooterMotor.set(
      leftMotorPIDController.calculate(
        leftShooterMotorAnlog.getVelocity(), leftRPM
      ));


    
  }

  public double controlSpeedMotorFeedForward(){
    return 0;
  }

  public Command shootingRPMCommand(double leftRPM, double rightRPM){
    // System.out.println("got here");
    // return new RunCommand(() -> shootingRPM(leftRPM, rightRPM), this);
    
    // return new ProxyCommand(
    //   () -> new TrapezoidProfileCommand(
    //     new TrapezoidProfile(SHOOTER_CONSTRAINTS),
    //     state -> controlSpeedMotorFeedForward(),
    //     () -> new TrapezoidProfile.State(0, leftRPM),
    //     () -> getCurrentState(),
    //     this));

    return new RunCommand(
      () -> shootingRPM(leftRPM, rightRPM),
      this);



  }

  public Command shootingRPMCommand(){
    return shootingRPMCommand(0, 0);
  }



}
