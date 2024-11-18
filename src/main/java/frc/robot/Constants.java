package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class Constants {
    public static final class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.2)
                .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(3).withKI(0).withKD(0)
                .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 150.0;

        // Initial configs for the drive and steer motors and the CANcoder; these cannot
        // be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API
        // documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            // Swerve azimuth does not require much torque output, so we can
                            // set a relatively low
                            // stator current limit to help avoid brownouts without
                            // impacting performance.
                            .withStatorCurrentLimit(60)
                            .withStatorCurrentLimitEnable(true));
        private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 5.21;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 12.8;
        private static final double kWheelRadiusInches = 2;

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "";
        private static final int kPigeonId = 13;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANbusName(kCANbusName)
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withCANcoderInitialConfigs(cancoderInitialConfigs);

        // Front Left
        private static final int kFrontLeftDriveMotorId = 1;
        private static final int kFrontLeftSteerMotorId = 5;
        private static final int kFrontLeftEncoderId = 9;
        private static final double kFrontLeftEncoderOffset = -0.454833984375;
        private static final boolean kFrontLeftSteerInvert = false;

        private static final double kFrontLeftXPosInches = 13.5;
        private static final double kFrontLeftYPosInches = 13.5;

        // Front Right
        private static final int kFrontRightDriveMotorId = 3;
        private static final int kFrontRightSteerMotorId = 7;
        private static final int kFrontRightEncoderId = 11;
        private static final double kFrontRightEncoderOffset = 0.274658203125;
        private static final boolean kFrontRightSteerInvert = false;

        private static final double kFrontRightXPosInches = 13.5;
        private static final double kFrontRightYPosInches = -13.5;

        // Back Left
        private static final int kBackLeftDriveMotorId = 2;
        private static final int kBackLeftSteerMotorId = 6;
        private static final int kBackLeftEncoderId = 10;
        private static final double kBackLeftEncoderOffset = 0.0498046875;
        private static final boolean kBackLeftSteerInvert = false;

        private static final double kBackLeftXPosInches = -13.5;
        private static final double kBackLeftYPosInches = 13.5;

        // Back Right
        private static final int kBackRightDriveMotorId = 4;
        private static final int kBackRightSteerMotorId = 8;
        private static final int kBackRightEncoderId = 12;
        private static final double kBackRightEncoderOffset = 0.277099609375;
        private static final boolean kBackRightSteerInvert = false;

        private static final double kBackRightXPosInches = -13.5;
        private static final double kBackRightYPosInches = -13.5;

        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide)
            .withSteerMotorInverted(kFrontLeftSteerInvert);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
            kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
            Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide)
            .withSteerMotorInverted(kFrontRightSteerInvert);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide)
            .withSteerMotorInverted(kBackLeftSteerInvert);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
            kInvertRightSide)
            .withSteerMotorInverted(kBackRightSteerInvert);

        public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants,
            FrontLeft,
            FrontRight, BackLeft, BackRight);
    }

    // Should we add any constants from `TunerConstants.java` into here to have all our constants in one place?
    public static final class IOConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDriveDeadband = 0.05;
    }
    public static final class LEDConstants {
        public static final int kCanID = 59;
        public static final int kStripLength = 15;
    }
    public static final class AutonConstants {
        public static final double kMaxSpeedMetersPerSeconds = 3.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

        public static final double kXP = 1.0;
        public static final double kXI = 0.0;
        public static final double kXD = 0.0;
        public static final double kYP = 1.0;
        public static final double kYI = 0.0;
        public static final double kYD = 0.0;

        public static final double kSubwooferTopRPM = 2400.0;
        public static final double kLowerMotorTargetRPM = 3000.0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularAccelerationRadiansPerSecondSquared, kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRPM = 5676.0;
    }

    public static final class ShooterConstants {
        public static final int kConveyorMotorCanId = 14;
        public static final int kIntakeMotorCanId = 13;
        public static final int kLowerMotorCanId = 15;
        public static final int kUpperMotorCanId = 16;
        public static final int kDistanceSensorId = 1;

        public static final int kConveyorMotorCurrentLimit = 60;
        public static final int kIntakeMotorCurrentLimit = 60;
        public static final int kLowerMotorCurrentLimit = 55;
        public static final int kUpperMotorCurrentLimit = 55;

        public static final double kUpperMotorP = 1.0;
        public static final double kUpperMotorI = 0.0;
        public static final double kUpperMotorD = 0.0;

        public static final double kLowerMotorP = 1.0;
        public static final double kLowerMotorI = 0.0;
        public static final double kLowerMotorD = 0.0;

        public static final double kTolerance = 30.0;
        public static final double kSteadySpeedRPM = 2500;

        public static final double kDistanceThreshold = 1.2;

        public static final double kFullSendVoltageScale = 1.2;

        public static final int kDistanceSensorAverageBits = 2;

        public static final double intakeEffort = -0.2;
        public static final double conveyorEffort = 0.2;
    }

    public static final class ArmConstants {
        public static final int kArmMotorCanId = 1;
        public static final double kArmMotorP = 0.2;
        public static final double kArmMotorI = 0.0;
        public static final double kArmMotorD = 0.0;
    }
}
