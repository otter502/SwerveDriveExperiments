package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.Constants.Drive.ModuleConstants;
import frc.robot.util.CANBaseLogger;

import static frc.robot.Constants.Drive.Modules.*;
import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.LoggingConstants.*;




public class SwerveModule {
    private final CANSparkBase mDriveMotor;
    private final CANSparkBase mAngleMotor;

    private final SparkPIDController mDrivePID;
    private final SparkPIDController mAnglePID;

    private final SparkRelativeEncoder mDriveEncoder;

    private final CANcoder mCANCoder;

    private final Translation2d position;

    private SwerveModuleState mLastDesiredStateSet;


    //init functions

    public SwerveModule(
        int driveMotorID,
        int angleMotorID,
        int encoderID,
        double encoderOffsetDegrees,
        double x,
        double y,
        String loggingName) 
    {
        
        mCANCoder = new CANcoder(encoderID);
        mCANCoder
                .getConfigurator()
                .apply(new CANcoderConfiguration()
                        .withMagnetSensor(new MagnetSensorConfigs()
                                .withMagnetOffset(-Rotation2d.fromDegrees(encoderOffsetDegrees).getRotations())));

        mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        configureDriveMotor(mDriveMotor);
        configureAngleMotor(mAngleMotor);

        mDrivePID = mDriveMotor.getPIDController();
        mAnglePID = mAngleMotor.getPIDController();

        new CANBaseLogger(mAngleMotor, kSwerveTab+"/"+loggingName, "AngleMotor");
        new CANBaseLogger(mDriveMotor, kSwerveTab+"/"+loggingName, "DriveMotor");
        

        mDriveEncoder = (SparkRelativeEncoder) mDriveMotor.getEncoder();

        position = new Translation2d(x, y);
    }

    public SwerveModule(ModuleConstants mc){
        this(
            mc.driveMotorID(), 
            mc.angleMotorID(),
            mc.encoderID(), 
            mc.encoderOffsetDegrees(),
            mc.x(), mc.y(),
            mc.loggingName()
        );
    }

    private void configureDriveMotor(final CANSparkBase motor) {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(kDriveMotorCurrentLimit);
        motor.enableVoltageCompensation(kMaxVoltage);
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake); // TODO: change this to brake after testing

        final var encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(kDrivePositionConversionFactor);
        encoder.setVelocityConversionFactor(kDriveVelocityConversionFactor);
        encoder.setPosition(0);

        final var pid = motor.getPIDController();
        final var pc = kDrivePIDFConstants;
        pid.setP(pc.p(), 1);
        pid.setI(pc.i(), 1);
        pid.setD(pc.d(), 1);
        pid.setFF(pc.ff(), 1);
        pid.setSmartMotionMaxVelocity(kMaxSpeed, 1);
        pid.setSmartMotionMaxAccel(11, 1);

        motor.burnFlash();
    }

    private void configureAngleMotor(final CANSparkBase motor) {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(kAngleMotorCurrentLimit);
        motor.enableVoltageCompensation(kMaxVoltage);
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);

        final var encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(kAnglePositionConversionFactor);
        encoder.setPosition(getAngle().getRadians());

        final var pid = motor.getPIDController();
        final var pc = kAnglePIDFConstants;
        pid.setP(pc.p());
        pid.setI(pc.i());
        pid.setD(pc.d());
        pid.setFF(pc.ff());
        pid.setPositionPIDWrappingEnabled(true);
        pid.setPositionPIDWrappingMaxInput(Math.PI);
        pid.setPositionPIDWrappingMinInput(-Math.PI);

        motor.burnFlash();
    }

    //general methods

    public void setDesiredState(SwerveModuleState state) {
        mDrivePID.setReference(state.speedMetersPerSecond, ControlType.kSmartVelocity, 1);
        mAnglePID.setReference(state.angle.getRadians(), ControlType.kPosition);

        mLastDesiredStateSet = state;
    }

    public SwerveModuleState getDesiredState() {
        return mLastDesiredStateSet;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(mDriveEncoder.getVelocity(), getAngle());
    }

    private Rotation2d getAngle() {
        if (RobotBase.isSimulation()) {
            return simAngle; 
        }
        return Rotation2d.fromRotations(mCANCoder.getPosition().getValue());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(mDriveEncoder.getPosition(), getAngle());
    }

    //Simulation Methods
    
    private Rotation2d simAngle = new Rotation2d();

    public void setPosition(SwerveModulePosition newPos){
        if (!RobotBase.isSimulation()) {
            throw new IllegalStateException("ran simulation specific method when real");
        }

        mDriveEncoder.setPosition(newPos.distanceMeters);
        simAngle = newPos.angle;
    }
}
