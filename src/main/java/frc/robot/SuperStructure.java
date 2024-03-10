package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakePositionSubsystem;

import static frc.robot.Constants.ClimbingConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class SuperStructure {
    private final IntakePositionSubsystem intakePositionSubsystem;
    private final IntakeFeederSubsystem intakeFeederSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final ClimbingSubsystem climbingSubsystem;
    public final Swerve SwerveSubsystem;

    // private final ClimbingSubsystem climbingSubsystem;

    private final Trigger endIntakeTrigger;

    public SuperStructure(Trigger endIntakeTrigger) {
        intakePositionSubsystem = new IntakePositionSubsystem();
        intakeFeederSubsystem = new IntakeFeederSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climbingSubsystem = new ClimbingSubsystem();
        SwerveSubsystem = new Swerve();


        this.endIntakeTrigger = endIntakeTrigger;

    }

    /* swerve drive Commands*/
    public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationSpeed,
            BooleanSupplier isOpenLoop) {
        return SwerveSubsystem.driveCommand(
                xSpeed,
                ySpeed,
                rotationSpeed,
                isOpenLoop);
    }

    /* intake position PID Commands */
    public Command groundIntake() {
        return intakePositionSubsystem.moveToAngle(GROUND_INTAKE_ANGLE)
                .raceWith(new WaitCommand(5));
    }

    public Command ampIntake() {
        return intakePositionSubsystem.moveToAngle(AMP_INTAKE_ANGLE)
                .raceWith(new WaitUntilCommand(endIntakeTrigger));
    }

    public Command closeIntake() {
        return intakePositionSubsystem.moveToAngle(SPEAKER_INTAKE_ANGLE)
                .raceWith(new WaitCommand(5));
    }

    /* non PID intake movment Commands */
    public Command moveIntakeUntillCurr(double speed, boolean direction) {
        return intakePositionSubsystem.startIntakeMovment(direction ? speed : -speed)
                .raceWith(new WaitCommand(INTAKE_POS_TIME));
    }
    public Command moveIntakeUntillAngle(double speed, double angle, boolean direction) {
        return intakePositionSubsystem.startIntakeMovment(direction ? speed : -speed)
                .raceWith(new WaitUntilCommand(
                        () -> intakePositionSubsystem.isIntakeInAngle(angle, direction)))
                .raceWith(new WaitUntilCommand(endIntakeTrigger));
    }
    public Command moveIntakeManualy(double speed) {
        return intakePositionSubsystem.runAngleMotorCommand(speed);
    }

    /* feeding Commands */
    public Command shootIntake(double speed) {
        return intakeFeederSubsystem.feedingCommand(speed);
    }
    public Command stopIntake() {
        return new InstantCommand(() -> intakeFeederSubsystem.feeding(0));
    }

    /* shooter Commands */
    public Command warmShooter(double QxShooterSpeedpotentiometer) {
        double Qxconvertion = (QxShooterSpeedpotentiometer + 1) * 0.5;
        return shooterSubsystem.shootingCommand(
                LEFT_MOTOR_SPEED_SPEAKER * Qxconvertion,
                RIGHT_MOTOR_SPEED_SPEAKER * Qxconvertion);
    }
    public Command closeShooter() {
        return new InstantCommand(() -> shooterSubsystem.setShootingSpeed(0, 0));
    }

    /* climbing Commands */
    public Command climb(DoubleSupplier climbingSpeed) {
        return climbingSubsystem.climbingCommand(climbingSpeed, climbingSpeed);
    }

    public class SuperStructureAutos {
        public Command shootingAuto() {
            return warmShooter(1)
                    .raceWith(new WaitCommand(2))
                    .andThen(shootIntake(INTAKE_SHOOTING_SPEED)
                            .raceWith(new WaitCommand(4)))
                    .andThen(closeShooter())
                    .andThen(stopIntake());
        }

        public Command taxiAuto() {
            return SwerveSubsystem.driveConstantSpeed(
                    -1,
                    0,
                    0,
                    2.5);
        }

        public Command shootAndScootAuto() {
            return shootingAuto()
                    .andThen(taxiAuto());

        }

        public Command testPathPlannerAuto() {
            return new PathPlannerAuto("TestAuto1");
        }

    }
}
