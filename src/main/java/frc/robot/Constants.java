package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final class ControllerConstants{
        public static final double STICK_DEADBAND = 0.05;

        public static final int DRIVER_CONTROLLER_ID = 1;
        public static final int OPERATOR_CONTROLLER_ID = 1;
    }


    public static final class ShooterConstants {
        public static final int LEFT_SHOOTER_MOTOR_ID = 3;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 2;

        public static final double LEFT_MOTOR_SPEED_SPEAKER = -7;
        public static final double RIGHT_MOTOR_SPEED_SPEAKER = -7;

        public static final double LEFT_MOTOR_SPEED_SLOW = -3;
        public static final double RIGHT_MOTOR_SPEED_SLOW = -3;

        public static final double LEFT_MOTOR_SPEED_SPEAKER_RPM = -700;
        public static final double RIGHT_MOTOR_SPEED_SPEAKER_RPM = -700;

        public static final double SHOOTING_ANGLE_FACTOR = 2;

        public static final TrapezoidProfile.Constraints SHOOTER_CONSTRAINTS = new TrapezoidProfile.Constraints(360, 720);

    }

    public static final class IntakeConstants {
        public static final double ANGLE_OFFSET = 170;
        public static final int velocityConversionFactor = 1;

        public static final double MIN_DEG = 0;
        public static final double MAX_DEG = 190;

        public static final int ANGLE_MOTOR_CURRENT_LIMIT = 30;
        public static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(360, 720);

        public static final int FEEDING_MOTOR_ID = 6;
        public static final int ANGLE_MOTOR_ID = 5;

        public static final double GROUND_INTAKE_ANGLE = 7;
        public static final double AMP_INTAKE_ANGLE = 120;
        public static final double SPEAKER_INTAKE_ANGLE = 187;

        public static final double MANUAL_INTAKE_SPEED = 0.25;
        public static final double AUTOMATIC_INTAKE_SPEED = 0.8;

        public static final double INTAKE_FEEDING_SPEED = -0.4;
        public static final double INTAKE_SHOOTING_SPEED = 0.57;

        public static final double INTAKE_POS_CUR_LIMIT = 1;

        public static final double INTAKE_POS_TIME = 0.9;

        public static final double KG_ANGLE = 0.08;
        public static final double KV_ANGLE = 2.00;
        public static final double KS_ANGLE = 0.00;
        public static final double KA_ANGLE = 0.00;


    }

    public static final class ClimbingConstants {
        public static final int RIGHT_CLIMBING_MOTOR_ID = 8;
        public static final int LEFT_CLIMBING_MOTOR_ID = 9;

        public static final double CLIMBING_SPEED = 5;

        public static final int CLIMBING_MOTOR_CURR_LIMIT = 40;

        public static final double CLIMBING_CONVERSION_FACTOR = 0.25;
    }

    public static final class Swerve {
        public static final COTSTalonFXSwerveConstants CHOOSEN_MODULE = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.555; // TODO: This must be tuned to specific robot
        public static final double WHEELBASE = 0.436; // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = CHOOSEN_MODULE.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEELBASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEELBASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = CHOOSEN_MODULE.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = CHOOSEN_MODULE.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 35;
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = CHOOSEN_MODULE.angleKP;
        public static final double ANGLE_KI = CHOOSEN_MODULE.angleKI;
        public static final double ANGLE_KD = CHOOSEN_MODULE.angleKD;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.12; // TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32; // TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 13;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(144.05);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 21;
            public static final int ANGLE_MOTOR_ID = 22;
            public static final int CANCODER_ID = 23;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-97.99 - 0.87);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 31;
            public static final int ANGLE_MOTOR_ID = 32;
            public static final int CANCODER_ID = 33;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-103.27);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 41;
            public static final int ANGLE_MOTOR_ID = 42;
            public static final int CANCODER_ID = 43;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(130);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
