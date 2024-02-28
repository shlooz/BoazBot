package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.*;
import static frc.robot.Constants.IntakeConstants.AUTOMATIC_INTAKE_SPEED;
import static frc.robot.Constants.IntakeConstants.INTAKE_FEEDING_SPEED;
import static frc.robot.Constants.IntakeConstants.INTAKE_SHOOTING_SPEED;
import static frc.robot.Constants.IntakeConstants.MANUAL_INTAKE_SPEED;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.controllers.controllers.QXDriveController;
import frc.robot.controllers.controllers.XboxOperationController;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final QXDriveController driver = new QXDriveController(DRIVER_CONTROLLER_ID);
    private final XboxOperationController operator = new XboxOperationController(OPERATOR_CONTROLLER_ID);

    /* Driver Buttons */

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final SuperStructure structure = new SuperStructure(operator.getClosedPositionButton());


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getXSpeed(), 
                () -> driver.getYSpeed(), 
                () -> driver.getRotationSpeed(), 
                () -> driver.getFieldOriented()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.getResetGyroButton().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driver.getModulesToAbsuluteButton().onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        /* operator PID controller */
        {
            /*
            operator.getClosedPositionButton().onTrue(structure.closeIntake());
            operator.getToAmpPositionButton().onTrue(structure.ampIntake());
            operator.getToGroundPositionButton().onTrue(structure.groundIntake());
            */

        }

        /* operatpr manual controll */
         {
              operator.getManualDownIntakeButton().whileTrue(structure.moveIntakeManualy(MANUAL_INTAKE_SPEED));
              operator.getManualDownIntakeButton().whileFalse(structure.moveIntakeManualy(0));
              operator.getManualUpIntakeButton().whileFalse(structure.moveIntakeManualy(0));
              operator.getManualUpIntakeButton().whileTrue(structure.moveIntakeManualy(-MANUAL_INTAKE_SPEED));
              

            operator.getToGroundPositionButton().onTrue(structure.moveIntakeUntillCurr(AUTOMATIC_INTAKE_SPEED, true));
            operator.getClosedPositionButton().onTrue(structure.moveIntakeUntillCurr(AUTOMATIC_INTAKE_SPEED, false));

            operator.getIntakeIntakingButton().whileTrue(structure.shootIntake(INTAKE_FEEDING_SPEED));
            operator.getIntakeIntakingButton().whileFalse(structure.shootIntake(0));
            operator.getIntakeOutakingButton().whileFalse(structure.shootIntake(0));
            operator.getIntakeOutakingButton().whileTrue(structure.shootIntake(INTAKE_SHOOTING_SPEED));

            operator.getWarmingButton().whileTrue(structure.warmShooter(driver.getShooterPotentiometer()));
            operator.getWarmingButton().whileFalse(structure.closeShooter());
        }
    
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    //     // An ExampleCommand will run in autonomous
    //     return Autos.shootingAuto();
    // }

    public void onEnable(){
        s_Swerve.resetModulesToAbsolute();
    }
}
