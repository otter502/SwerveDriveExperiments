package frc.robot.commands.driveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static edu.wpi.first.math.MathUtil.applyDeadband;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrive;
import static frc.robot.Constants.OI.kStickDeadband;

import java.util.Set;

import static frc.robot.Constants.Drive.*;

public class AngleDrivenTeleopDrive extends AngleDriveCommand {
    private SwerveDrive mDriveBase;
    private static final CommandXboxController dc = RobotContainer.getDriverController();
    public static boolean mIsFieldRelative = true; 

    public static void setFieldRelative(boolean fieldRelative){
        mIsFieldRelative = fieldRelative;
    }

    public static Command setFieldRelativeTrue(){
        return Commands.runOnce(() -> setFieldRelative(true));
    }

    public static Command setFieldRelativeFalse(){
        return Commands.runOnce(() -> setFieldRelative(false));
    }

    public AngleDrivenTeleopDrive(SwerveDrive driveBase) {
        super();
        mDriveBase = driveBase;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        ChassisSpeeds cs = new ChassisSpeeds();

        {
            final double inputxraw = dc.getLeftY() * -1.0;
            final double inputyraw = dc.getLeftX() * -1.0;

            final double inputx = applyResponseCurve(applyDeadband(inputxraw, kStickDeadband));
            final double inputy = applyResponseCurve(applyDeadband(inputyraw, kStickDeadband));

            cs.vxMetersPerSecond = inputx * kMaxSpeed;
            cs.vyMetersPerSecond = inputy * kMaxSpeed;
        }

        Rotation2d desiredAngle;
        {
            final double inputxraw = dc.getLeftY() * -1.0;
            final double inputyraw = dc.getLeftX() * -1.0;

            final double inputx = applyResponseCurve(applyDeadband(inputxraw, kStickDeadband));
            final double inputy = applyResponseCurve(applyDeadband(inputyraw, kStickDeadband));

            desiredAngle = new Rotation2d(inputx, inputy);
        }

        drive(cs.vxMetersPerSecond, cs.vyMetersPerSecond, desiredAngle);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    private double applyResponseCurve(double x) {
        return Math.signum(x) * Math.pow(x, 2);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(mDriveBase); 
    }
}
