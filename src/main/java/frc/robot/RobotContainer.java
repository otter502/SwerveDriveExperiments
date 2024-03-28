// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.tempSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.util.CANBaseLogger;

import static frc.robot.Constants.OI.*;
public class RobotContainer {
    private static final CommandXboxController mDriverController = new CommandXboxController(kDriverControllerPort);
    private static final CommandXboxController mOperatorController = new CommandXboxController(kOperatorControllerPort);

    public static CommandXboxController getDriverController() {
        return mDriverController;
    }
    public static CommandXboxController getOperatorController() {
        return mOperatorController;
    }

    // private final SwerveDrive mSwerveDrive = SwerveDrive.getInstance(); 
    private final tempSubsystem testing = new tempSubsystem();
    public static Pose2d getStartingPose(){
        return new Pose2d();
    }

    public RobotContainer() {
        if (RobotBase.isSimulation()) {
            new SwerveSim();
        }

        //testing
        configureBindings();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
