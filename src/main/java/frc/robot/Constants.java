package frc.robot;

import java.util.stream.Stream;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDFConstants;

public class Constants {
    public static final class OI {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kStickDeadband = 0.05;

        public static final double kDriverTriggerDeadband = 0.3;
        public static final double kOperatorTriggerDeadband = 0.3;
    }
    
    public static final class Drive{
        public static final class Modules{
            public static final PIDFConstants kDrivePIDFConstants = new PIDFConstants(0.0, 0.0, 0.0, 0.2); // placholdersvalues
            public static final PIDFConstants kAnglePIDFConstants = new PIDFConstants(1.0, 0.0, 0.5, 0.0); //placeholders

            public static final double kDriveGearRatio = 6.75 / 1.0; // L2 MK4i
            public static final double kDriveVelocityConversionFactor = ((kWheelDiameter * Math.PI) / kDriveGearRatio) / 60.0; // RPM to m/s
            public static final double kDrivePositionConversionFactor = ((kWheelDiameter * Math.PI) / kDriveGearRatio); // rev to meters
    
            public static final double kAngleGearRatio = 150.0 / 7.0; // MK4i
            public static final double kAnglePositionConversionFactor = (2 * Math.PI) / (kAngleGearRatio);

            
        public static final int kDriveMotorCurrentLimit = 40;
            public static final int kAngleMotorCurrentLimit = 25;
            public static final double kMaxVoltage = 12.0;
        }


        public static final int kPigeon2ID = 2;

        public static final Matrix<N3, N1> kStateStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> kVisionMeasurementStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.1);

        public static final double kWheelDiameter = Units.inchesToMeters(3.96);
        public static final double kTrackWidth = Units.inchesToMeters(22.475);
        public static final double kWheelbase = Units.inchesToMeters(22.475);
        public static final double kChassisRadius = Math.hypot(
                kTrackWidth / 2, kWheelbase / 2);

        // theoretical maximum with NEO and L2 MK4i
        public static final double kMaxSpeed = Units.feetToMeters(15.1);
        public static final double kMaxAcceleration = 5; // 9.81 * kTreadCoefficientOfFriction * kTreadWearAdjustment;

        public static final double kMaxAngularSpeed = kMaxSpeed / kChassisRadius;
        public static final double kMaxAngularAcceleration = kMaxAcceleration / kChassisRadius;
        
        // Decrease this value if wheels start to slip with worn out tread. Should be 1.0 with new tread.
        public static final double kTreadWearAdjustment = 1.0;
        public static final double kTreadCoefficientOfFriction = 1.13; // black neoprene


        public static final ModuleConstants kFrontLeft = new ModuleConstants(
                3, 4, 11, -91.318, kWheelbase / 2, kTrackWidth / 2, "FrontLeft");

        public static final ModuleConstants kBackLeft = new ModuleConstants(
                5, 6, 12, 74.268, -kWheelbase / 2, kTrackWidth / 2, "BackLeft");

        public static final ModuleConstants kBackRight = new ModuleConstants(
                7, 8, 13, 286.699, -kWheelbase / 2, -kTrackWidth / 2, "BackRight");

        public static final ModuleConstants kFrontRight = new ModuleConstants(
                9, 10, 14, 166.377, kWheelbase / 2, -kTrackWidth / 2, "FrontRight");

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
                (Translation2d[]) Stream.of(new ModuleConstants[] { kFrontLeft, kBackLeft, kBackRight, kFrontRight })
                        .map((mc) -> new Translation2d(mc.x(), mc.y()))
                        .toArray(Translation2d[]::new));

        public static final double kOdometryUpdatePeriod = 0.01;
        public static final double kDriveUpdatePeriod = 0.01;


        
        public static final record ModuleConstants(
                int driveMotorID,
                int angleMotorID,
                int encoderID,
                double encoderOffsetDegrees,
                double x,
                double y,
                String loggingName) {}
    }

    public static final class AdvScopeLogging {
        public static final String kLoggingTabName = "Logging";
        public static final String kSwerveTab = "Swerve"; //Logging/Swerve

    }
}
