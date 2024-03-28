package frc.robot.subsystems.swerve;

import java.util.List;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.stream.Stream;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.driveCommands.TeleopDriveCommand;

import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.AdvScopeLogging.*;


public class SwerveDrive extends SubsystemBase{

    private static SwerveDrive mInstance;

    private final SwerveModule mFrontLeft;
    private final SwerveModule mBackLeft;
    private final SwerveModule mBackRight;
    private final SwerveModule mFrontRight;

    private final SwerveModule[] mModules;
    
    private final Pigeon2 mIMU;

    private final SwerveDrivePoseEstimator mPoseEstimator;
    private final Notifier mOdometryThread = new Notifier(this::updateOdometry);
    private ReadWriteLock mPoseEstimationRWLock = new ReentrantReadWriteLock();


    public SwerveDrive(
        Pose2d initialPose
    ) {
        
        mFrontLeft = new SwerveModule(kFrontLeft);
        mBackLeft = new SwerveModule(kBackLeft);
        mBackRight = new SwerveModule(kBackRight);
        mFrontRight = new SwerveModule(kFrontRight);

        mModules = new SwerveModule[] { mFrontLeft, mBackLeft, mBackRight, mFrontRight };

        mIMU = new Pigeon2(kPigeon2ID);
        mIMU.reset();
        

        {
            final var stateStdDevs = kStateStdDevs;
            final var visionMeasurementStdDevs = kVisionMeasurementStdDevs;

            // TODO: change initial pose estimate
            mPoseEstimator = new SwerveDrivePoseEstimator(kKinematics, getAngle(), getModulePositions(), initialPose, stateStdDevs, visionMeasurementStdDevs);
        }

        mOdometryThread.startPeriodic(kOdometryUpdatePeriod);

        // new Trigger(DriverStation::isDisabled)
        //         .onTrue(Commands.runOnce(this::setCoastMode))
        //         .onFalse(Commands.runOnce(this::setBrakeMode));

        setDefaultCommand(new TeleopDriveCommand(this));

    }

    public static SwerveDrive getInstance(){
        if (mInstance == null) {
            mInstance = new SwerveDrive(RobotContainer.getStartingPose());
        }
        return mInstance;
    }

    //Component functions

    private SwerveModulePosition[] getModulePositions() {
        return Stream.of(mModules).map((m) -> m.getPosition()).toArray(SwerveModulePosition[]::new);
    }
    
    public SwerveModuleState[] getModuleDesiredStates() {
        return (SwerveModuleState[]) Stream.of(mModules).map((m) -> m.getDesiredState()).toArray(SwerveModuleState[]::new);
    }

    public SwerveModuleState[] getModuleCurrentStates() {
        return (SwerveModuleState[]) Stream.of(mModules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new);
    }

    public SwerveDriveKinematics getKinematics(){
        return kKinematics;
    }
    
    public Pigeon2 getIMU(){
        return mIMU;
    }

    public SwerveModule[] getModules(){
        return mModules;
    }

    //Odometry

    /**
     * Gets the angle from the IMU.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(mIMU.getAngle() * -1.0);
    }

    public Pose2d getPose(){
        Pose2d out;
        mPoseEstimationRWLock.readLock().lock();
        try {
            out = mPoseEstimator.getEstimatedPosition();
        } catch (Exception e) {
            throw e;
        } finally{
            mPoseEstimationRWLock.readLock().unlock();
        }
        return out;
    }

    public void resetAngle(){
        mIMU.reset();
    }

    /**
     * Adds a vision measurement to the pose estimator.
     */
    public void addVisionMeasurement(final Pose2d cameraPose, final double t) {
        mPoseEstimationRWLock.writeLock().lock();

        //TODO add more complicated logic here
    
        try {
            mPoseEstimator.addVisionMeasurement(cameraPose, t);
        } catch (Exception e) {
            throw e;
        } finally{
            mPoseEstimationRWLock.writeLock().unlock();
        }
    }


    private void updateOdometry() {
        var angle = getAngle();

        mPoseEstimationRWLock.writeLock().lock();

        try {
            mPoseEstimator.update(angle, getModulePositions());
        } catch (Exception e){
            throw e;
        }finally{
            mPoseEstimationRWLock.writeLock().unlock();
        }
    }

    //drive functions

    public void drive(final ChassisSpeeds speeds, final boolean fieldRelative) {
        drive(speeds, fieldRelative, false);
    }

    public void driveRobotRelative(final ChassisSpeeds speeds) {
        drive(speeds, false);
    }

    // FIXME: need to make field relative based on pose and mirror based on alliance
    public void drive(final ChassisSpeeds speeds, final boolean fieldRelative, final boolean discretizeSpeeds) {

        ChassisSpeeds _speeds;
        if (fieldRelative) {
            _speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
        } else {
            _speeds = speeds;
        }
        if (discretizeSpeeds){
            _speeds = ChassisSpeeds.discretize(_speeds, kDriveUpdatePeriod);
        }

        final var moduleStates = kKinematics.toSwerveModuleStates(_speeds);

        final var moduleStatesIterator = List.of(moduleStates).iterator();

        for (SwerveModule m : mModules) {
            m.setDesiredState(SwerveModuleState.optimize(moduleStatesIterator.next(), m.getState().angle));
        }
    }

    //logging

    private StructArrayPublisher<SwerveModuleState> desiredStatePublisher = NetworkTableInstance.getDefault().getTable(kLoggingTabName).getSubTable(kSwerveTab).getStructArrayTopic("SwerveModulesDesiredStates", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> currentStatePublisher = NetworkTableInstance.getDefault().getTable(kLoggingTabName).getSubTable(kSwerveTab).getStructArrayTopic("SwerveModulesCurrentStates", SwerveModuleState.struct).publish();
    private StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault().getTable(kLoggingTabName).getSubTable(kSwerveTab).getStructTopic("robotPose", Pose2d.struct).publish();

    @Override
    public void periodic() {
        updateOdometry(); //CHECKUP is this needed
        
        desiredStatePublisher.accept(getModuleDesiredStates());
        currentStatePublisher.accept(getModuleCurrentStates());
        robotPosePublisher.accept(getPose());
    }
}
