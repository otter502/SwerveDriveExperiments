package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CANBaseLogger;

public class tempSubsystem extends SubsystemBase{
    CANSparkMax mMotor;
    GenericEntry io;

    public tempSubsystem() {
        super();
        mMotor = new CANSparkMax(1, MotorType.kBrushless);
        mMotor.setIdleMode(IdleMode.kCoast);
        new CANBaseLogger(mMotor, "TestingLogger", "mainMotor");
        var tab = Shuffleboard.getTab("testing");
        io = tab.add("setMotor", 0.0).getEntry();
    }
    
    @Override
    public void periodic() {
        mMotor.set(io.getDouble(0.0));
    }
}
