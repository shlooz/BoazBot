// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Util.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

enum position {
  GROUND,
  AMP,
  SHOOTER
}


public class IntakePositionSubsystem extends SubsystemBase {
  // private static final double INTAKE_POS_CUR_LIMIT = 0;
  /** Creates a new intakePositionSubsystem. */
  static CANSparkMax angleMotor;
  SparkPIDController angleController;
  AbsoluteEncoder angleAbsoluteEncoder;

  position intakePose;

  double targetAngle;
  boolean stopIntakeFlag;
  boolean supp;
  double currAngle;
  double toPrintFeedForward;

  public IntakePositionSubsystem() {
    angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, MotorType.kBrushless);
    angleMotor.restoreFactoryDefaults();

    angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) (MAX_DEG));
    angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) (MIN_DEG));

    angleMotor.setSmartCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);
    angleMotor.setInverted(true);

    angleAbsoluteEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    angleAbsoluteEncoder.setPositionConversionFactor(360);
    angleAbsoluteEncoder.setZeroOffset(ANGLE_OFFSET);
    angleAbsoluteEncoder.setVelocityConversionFactor(velocityConversionFactor);


    angleController = angleMotor.getPIDController();
    angleController.setFeedbackDevice(angleAbsoluteEncoder);
    angleMotor.setClosedLoopRampRate(0.5);
    updateSparkMaxPID(angleController, 0.01, 0, 0.3, 0, 0, -1, 1);

    targetAngle = 0;
    stopIntakeFlag = false;
    supp = false;
    toPrintFeedForward = 0;
  }

  @Override
  public void periodic() {
    intakePose = getIntakeAngle() < 10 ?
      position.GROUND : getIntakeAngle() < 150 ?
        position.AMP : position.SHOOTER;
  }

  // This method will be called once per scheduler run

  public static void enableCoast(boolean enable) {
    if (enable) {
      angleMotor.setIdleMode(IdleMode.kCoast);
    } else {
      angleMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  public boolean isCurrFine() {
    return (angleMotor.getOutputCurrent() < INTAKE_POS_CUR_LIMIT);
  }

  public void setMotor(double speed) {
    angleMotor.set(speed);
  }

  public Command startIntakeMovment(double speed) {
    RunCommand command = new RunCommand(() -> {
      stopIntakeFlag = true;
      setMotor(speed);
    });
    // return this.runOnce(() -> {
    // stopIntakeFlag = true;
    // setMotor(speed);
    // });

    return command;
  }

  public void stopIntakeMovment() {
    stopIntakeFlag = false;
    setMotor(0);
  }

  public Command runAngleMotorCommand(double speed) {
    return new RunCommand(() -> setMotor(speed), this);
  }

  public Command moveIntakeOnCurrent(double speed) {
    return new RunCommand(() -> {
      while (angleMotor.getOutputCurrent() < INTAKE_POS_CUR_LIMIT) {
        setMotor(speed);
      }
      setMotor(0);
    });
  }

  public Command printing(String message) {
    return new RunCommand(() -> System.out.println(message));
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(
        angleAbsoluteEncoder.getPosition(),
        angleAbsoluteEncoder.getVelocity());
  }

  public boolean isIntakeDown(){
    return intakePose == position.GROUND ?
        true : false;
  }

  public boolean isIntakeMid(){
    return intakePose == position.AMP ?
        true : false;
  }

  public boolean isIntakeUP(){
    return intakePose == position.SHOOTER ?
        true : false;
  }

  public double getIntakeAngle() {
    return angleAbsoluteEncoder.getPosition();
  }

  public boolean isIntakeInAngle(double angle, boolean direction) {
    return (direction ? getIntakeAngle() < angle || getIntakeAngle() > 300
        : getIntakeAngle() > angle && getIntakeAngle() < 300);
  }

  /**
   * @param angle
   * @return
   */
  public Command moveToAngle(double angle) {
    /*
     * return new ProxyCommand(
     * () -> new TrapezoidProfileCommand(
     * new TrapezoidProfile(
     * ANGLE_CONSTRAINTS,
     * new TrapezoidProfile.State(angle, 0),
     * new TrapezoidProfile.State(angleAbsoluteEncoder.getPosition(), 0)
     * ),
     * state -> {
     * double feedforward = 0;
     * // kS_ANGLE * Math.signum(state.velocity)
     * // + kG_ANGLE.get(lengthEncoder.getPosition()) * Math.cos(state.position)
     * // + kV_ANGLE * state.velocity;
     * angleController.setReference(state.position,
     * CANSparkMax.ControlType.kPosition,
     * 0,
     * feedforward, SparkPIDController.ArbFFUnits.kVoltage);
     * }
     * 
     * 
     * ));
     */

    return new ProxyCommand(() -> new TrapezoidProfileCommand(
        new TrapezoidProfile(ANGLE_CONSTRAINTS),
        state -> controlAngleMotor(state),
        () -> new TrapezoidProfile.State(angle, 0),
        () -> getCurrentState(),
        this));

    // return new RunCommand(
    // () -> {
    // //System.out.println("Target Angle: " + angle);
    // angleController.setReference(angle, CANSparkMax.ControlType.kPosition);},
    // this
    // ).asProxy();

  }

  private void controlAngleMotor(State state) {
    double feedForward = KS_ANGLE * Math.signum(Math.toRadians(state.velocity))
        + KG_ANGLE * Math.cos(Math.toRadians(getCurrentState().position))
        + KV_ANGLE * Math.toRadians(state.velocity);
    angleController.setReference(state.position, CANSparkMax.ControlType.kPosition,
        0,
        feedForward, SparkPIDController.ArbFFUnits.kVoltage);

  }

  public Command moveToAngle() {
    return moveToAngle(0);
  }

}
