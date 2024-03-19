package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    public final Swerve swerveSubsystem;

    // private final ClimbingSubsystem climbingSubsystem;

    private final Trigger endIntakeTrigger;

    public SuperStructure(Trigger endIntakeTrigger) {
        intakePositionSubsystem = new IntakePositionSubsystem();
        intakeFeederSubsystem = new IntakeFeederSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        climbingSubsystem = new ClimbingSubsystem();
        swerveSubsystem = new Swerve();

        this.endIntakeTrigger = endIntakeTrigger;

    }

    /* swerve drive Commands */
    public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationSpeed,
            BooleanSupplier isFieldOriented) {
        return swerveSubsystem.driveCommand(
                xSpeed,
                ySpeed,
                rotationSpeed,
                isFieldOriented);
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
    public Command warmShooter() {
        System.out.println("got here 0");
        return shooterSubsystem.shootingCommand(
                LEFT_MOTOR_SPEED_SPEAKER,
                RIGHT_MOTOR_SPEED_SPEAKER);
    }

    public Command warmShooterWithAngle() {
        DoubleSupplier robotsOrientation = () -> swerveSubsystem.getRobotOrientationForSpeaker();
        System.out.println(robotsOrientation.getAsDouble());
        if (robotsOrientation.getAsDouble() == -1) {
            return shooterSubsystem.shootingCommand(
                    LEFT_MOTOR_SPEED_SPEAKER,
                    RIGHT_MOTOR_SPEED_SPEAKER );
        } else if (robotsOrientation.getAsDouble() == 0) {
            return warmShooter();
        } else if (robotsOrientation.getAsDouble() == 1) {
            return shooterSubsystem.shootingCommand(
                    LEFT_MOTOR_SPEED_SPEAKER * SHOOTING_ANGLE_FACTOR,
                    RIGHT_MOTOR_SPEED_SPEAKER);
        }
        return null;

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
            return warmShooter()
                    .raceWith(new WaitCommand(2))
                    .andThen(shootIntake(INTAKE_SHOOTING_SPEED)
                            .raceWith(new WaitCommand(4)))
                    .andThen(closeShooter())
                    .andThen(stopIntake());
        }

        public Command taxiAuto() {
            return swerveSubsystem.driveConstantSpeed(
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
