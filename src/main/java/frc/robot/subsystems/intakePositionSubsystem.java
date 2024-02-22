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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;


public class IntakePositionSubsystem extends SubsystemBase {
  /** Creates a new intakePositionSubsystem. */
    CANSparkMax angleMotor;
    SparkPIDController angleController;
    AbsoluteEncoder angleAbsoluteEncoder;
  
  public IntakePositionSubsystem() {
      angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, MotorType.kBrushless);
      angleMotor.restoreFactoryDefaults();
      angleMotor.setClosedLoopRampRate(0.5);
      //angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      //angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      //angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)(MAX_DEG));
      //angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)(MIN_DEG));
      angleMotor.setIdleMode(IdleMode.kBrake);
      angleMotor.setSmartCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);

      angleAbsoluteEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
      angleAbsoluteEncoder.setPositionConversionFactor(360);
      angleAbsoluteEncoder.setZeroOffset(ANGLE_OFFSET);
      angleAbsoluteEncoder.setVelocityConversionFactor(velocityConversionFactor);

      angleController = angleMotor.getPIDController();
      angleController.setFeedbackDevice(angleAbsoluteEncoder);
      updateSparkMaxPID(angleController, 0.12, 0, 0, 0, 0, -1, 1);
  }

  
  @Override
  public void periodic() {
    System.out.println(angleAbsoluteEncoder.getPosition());
    // This method will be called once per scheduler run
  }
  
  public void setMotor(double speed){
    angleMotor.set(speed);
  }


  public TrapezoidProfile.State getCurrentState(){
    return new TrapezoidProfile.State(
      angleAbsoluteEncoder.getPosition(),
      angleAbsoluteEncoder.getVelocity()
    );
  }

  /**
   * @param angle
   * @return
   */
  public Command moveToAngle(double angle){
    /* return new ProxyCommand(
        () -> new TrapezoidProfileCommand(
                new TrapezoidProfile(
                        ANGLE_CONSTRAINTS,
                        new TrapezoidProfile.State(angle, 0),
                        new TrapezoidProfile.State(angleAbsoluteEncoder.getPosition(), 0)
                ),
                state -> {
                    double feedforward = 0;
                            // kS_ANGLE * Math.signum(state.velocity)
                            //         + kG_ANGLE.get(lengthEncoder.getPosition()) * Math.cos(state.position)
                            //         + kV_ANGLE * state.velocity;
                    angleController.setReference(state.position, CANSparkMax.ControlType.kPosition,
                            0,
                            feedforward, SparkPIDController.ArbFFUnits.kVoltage);
                } 


          )); */
    
    return new ProxyCommand(() -> new TrapezoidProfileCommand(
        new TrapezoidProfile(ANGLE_CONSTRAINTS), 
        state -> controlAngleMotor(state),
        () -> new TrapezoidProfile.State(angle, 0),
        () -> getCurrentState(),
        this)
    );
              
  
    }


  private void controlAngleMotor(State state) {
     double feedforward = 0;
    // double feedForward = kS_ANGLE * Math.signum(state.velocity)
    //                                         + kG_ANGLE * Math.cos(state.position)
    //                                         + kV_ANGLE * state.velocity;
    angleController.setReference(state.position, CANSparkMax.ControlType.kPosition,
            0,
            feedforward, SparkPIDController.ArbFFUnits.kVoltage);
  }

  public Command moveToAngle(){
    return moveToAngle(0);
  }


}
