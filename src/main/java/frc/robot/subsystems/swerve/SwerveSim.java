package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Drive.kKinematics;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSim extends SubsystemBase {
    private SwerveDrive mSwerve;
    private SwerveDriveKinematics kinematics;
    private Pigeon2 IMU;

    double lastTime;

    public SwerveSim() {
        lastTime = 0;
        mSwerve = SwerveDrive.getInstance();
        kinematics = mSwerve.getKinematics();
        IMU = mSwerve.getIMU();
    }

    @Override
    public void simulationPeriodic() {
        
        double dT = Timer.getFPGATimestamp() - lastTime;

        if (lastTime == 0) {
            lastTime = Timer.getFPGATimestamp();
            return;
        } else {
            lastTime = Timer.getFPGATimestamp();
        }
        
        boolean validStates = true; 



        for (SwerveModule m : mSwerve.getModules()) {
            
            SwerveModuleState currDesiredState = m.getDesiredState();
            if (currDesiredState == null) {
                validStates = false;
                continue;
            }

            double newPos = m.getPosition().distanceMeters + dT * currDesiredState.speedMetersPerSecond;
            Rotation2d newAngle = currDesiredState.angle;
            m.setPosition(new SwerveModulePosition(newPos, newAngle));
        }

        if (!validStates) { //prevents null pointer exception
            return;
        }

        var statesList = mSwerve.getModuleDesiredStates();

        ChassisSpeeds currChassisSpeed = kinematics.toChassisSpeeds(statesList);

        Pigeon2SimState simState = IMU.getSimState();
        simState.addYaw(Rotation2d.fromRadians(currChassisSpeed.omegaRadiansPerSecond).getDegrees() * dT);

    }

}
