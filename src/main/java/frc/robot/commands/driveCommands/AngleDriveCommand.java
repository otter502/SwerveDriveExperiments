package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.PIDConstants;

/**
 * this is a command that directly controls the swerve drives field relative angle via its protected {@link #drive} method
 * this assumes that the speeds are field relative
 */
public class AngleDriveCommand extends Command{
    private static final PIDConstants kAnglePIDConstants = new PIDConstants(9.0, 0, 0);
    private PIDController mAnglePID = new PIDController(kAnglePIDConstants.p(), kAnglePIDConstants.i(), kAnglePIDConstants.d());

    private SwerveDrive mDriveBase;

    public AngleDriveCommand() {
        super();
        mDriveBase = SwerveDrive.getInstance();
    }
    
    public AngleDriveCommand(SwerveDrive mDriveBase) {
        super();
        this.mDriveBase = mDriveBase;
    }

    protected void drive(double vxMetersPerSecond, double vyMetersPerSecond, Rotation2d desiredAngle){
        var omegaSpeed = mAnglePID.calculate(mDriveBase.getAngle().getDegrees(), desiredAngle.getDegrees());

        mDriveBase.drive(new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaSpeed), true);

    }
}
