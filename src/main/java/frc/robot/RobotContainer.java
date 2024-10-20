package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ControllerConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.commands.*;
import frc.robot.controllers.controllers.QXDriveController;
import frc.robot.controllers.controllers.QXOperationController;
import frc.robot.controllers.controllers.XboxDriveController;
import frc.robot.controllers.controllers.XboxOperationController;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final QXDriveController driver = new QXDriveController(1);
    private final QXOperationController operator = new QXOperationController(OPERATOR_CONTROLLER_ID);

    // private final XboxDriveController driver = new XboxDriveController(0);
    // private final XboxOperationController operator = new XboxOperationController(0);

    /* Driver Buttons */

    /* Subsystems */
    private final SuperStructure structure = new SuperStructure(operator.getStopIntakeButton());
    private final SuperStructure.SuperStructureAutos Autos = structure.new SuperStructureAutos();

    SendableChooser<Command> auto_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        structure.swerveSubsystem.setDefaultCommand(
                structure.drive(
                        () -> -driver.getYSpeed(),
                        () -> -driver.getXSpeed(),
                        () -> driver.getRotationSpeed(),
                        () -> driver.getFieldOriented()
                )
            );

        auto_chooser.setDefaultOption("Shoot and Scoot", Autos.shootAndScootAuto());
        auto_chooser.addOption("Shoot", Autos.shootingAuto());
        auto_chooser.addOption("Taxi", Autos.taxiAuto());
        // auto_chooser.addOption("Test1", Autos.testPathPlannerAuto());
        auto_chooser.addOption("Do Nothing", new InstantCommand());

        // SmartDashboard.putData("Auto Chooser", auto_chooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.getResetGyroButton().onTrue(new InstantCommand(() -> structure.swerveSubsystem.zeroHeading()));
        // driver.getModulesToAbsuluteButton().onTrue(new InstantCommand(() ->
        // structure.SwerveSubsystem.resetModulesToAbsolute()));
        /* operator PID controller */
        {

            operator.getToClosedPositionButton().onTrue(structure.closeIntake());
            operator.getToAmpPositionButton().onTrue(structure.ampIntake());
            operator.getToGroundPositionButton().onTrue(structure.groundIntake());

        }

        /* operatpr manual controll */
        {

            // operator.getManualIntakePositionDown().whileTrue(structure.moveIntakeManualy(-MANUAL_INTAKE_SPEED));
            // operator.getManualIntakePositionDown().whileFalse(structure.moveIntakeManualy(0));
            // operator.getManualIntakePositionUp().whileFalse(structure.moveIntakeManualy(0));
            // operator.getManualIntakePositionUp().whileTrue(structure.moveIntakeManualy(MANUAL_INTAKE_SPEED));

            operator.getIntakeIntakingButton().and(()  -> structure.intakeState()).whileTrue(structure.shootIntake(INTAKE_FEEDING_SPEED));
            operator.getIntakeIntakingButton().and(() -> !structure.intakeState()).whileTrue(structure.shootIntake(INTAKE_SHOOTING_SPEED));
            operator.getIntakeIntakingButton().whileFalse(structure.shootIntake(0));


            operator.getWarmingButton().onTrue(structure.shootingSequence());
            // operator.getWarmingButton().whileFalse(structure.closeShooter());

        }

        //     operator.getClimbing().onTrue(
        //             structure.climb(() -> operator.getClimbingSpeed()));
        // }

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // PathPlannerPath path = PathPlannerPath.fromPathFile("TestAuto1");
        // return AutoBuilder.followPath(path);
        return Autos.shootAndScootAuto();
    }

    public void onEnable() {
        structure.swerveSubsystem.resetModulesToAbsolute();
    }

}