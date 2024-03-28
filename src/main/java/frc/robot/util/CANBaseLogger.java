package frc.robot.util;

import java.util.ArrayList;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import static frc.robot.Constants.AdvScopeLogging.kLoggingTabName;

public class CANBaseLogger {
    private final boolean VERBOSE_LOGGING = true;

    public static ArrayList<CANBaseLogger> loggers;

    private CANSparkBase motor;
    private RelativeEncoder relativeEncoder;
    private SparkPIDController pid;
    private NetworkTable loggingFolder;

    //structure:
    // CANBase:MotorName
    /**
     * the motor will be logged to the networktable at the following location: <p>
     * Logging/&lt BaseTableLocation &gt/CANBase:&lt motorName &gt <p>
     *      
     * @param motor the motor to be logged
     * @param baseTableLocation where the logger will log, it will automatically put it in the logging folder
     * @param motorName the name of the motor (unique)
     * 
     */
    public CANBaseLogger(CANSparkBase motor, String baseTableLocation, String motorName) {
        super();
        this.motor = motor;

        pid = motor.getPIDController();
        relativeEncoder = motor.getEncoder();
        loggingFolder = NetworkTableInstance.getDefault().getTable(kLoggingTabName + NetworkTable.normalizeKey(baseTableLocation, false) + "CANBase:"+motorName);

        appliedOuputPub = loggingFolder.getDoubleTopic("appliedOuput").publish();
        outputCurrentPub = loggingFolder.getDoubleTopic("outputCurrent").publish();
        relEncoderPosReadingPub = loggingFolder.getDoubleTopic("relEncPosition").publish();
        relEncoderVelReadingPub = loggingFolder.getDoubleTopic("relEncCurrent").publish();
        pidConstantsPub = loggingFolder.getDoubleArrayTopic("pidConstants").publish();
        iZoneReadingPub = loggingFolder.getDoubleTopic("iZoneReading").publish();
        motorFaultsPub = loggingFolder.getBooleanArrayTopic("motorFaults").publish();
        lastWarningPub = loggingFolder.getStringTopic("lastError").publish();
        motorTempraturePub = loggingFolder.getDoubleTopic("motorTemp").publish();

        loggers.add(this);
    }

    DoublePublisher appliedOuputPub;
    DoublePublisher outputCurrentPub;
    DoublePublisher relEncoderPosReadingPub;
    DoublePublisher relEncoderVelReadingPub;
    DoubleArrayPublisher pidConstantsPub;
    DoublePublisher iZoneReadingPub;
    BooleanArrayPublisher motorFaultsPub;
    StringPublisher lastWarningPub;
    DoublePublisher motorTempraturePub;



    public void updateLogs(){
        appliedOuputPub.accept(motor.getAppliedOutput());
        outputCurrentPub.accept(motor.getOutputCurrent());
        relEncoderPosReadingPub.accept(relativeEncoder.getPosition());
        relEncoderVelReadingPub.accept(relativeEncoder.getVelocity());
        pidConstantsPub.accept(new double[]{
            pid.getP(),
            pid.getI(),
            pid.getD(),
            pid.getFF()
        });
        iZoneReadingPub.accept(pid.getIAccum());
        

        if (!VERBOSE_LOGGING) {
            return;
        }

        lastWarningPub.accept(motor.getLastError().name());
        motorTempraturePub.accept(motor.getMotorTemperature());
        short faults = motor.getFaults();
        motorFaultsPub.accept(new boolean[]{
            ((faults & 1 << 0) == 1),
            ((faults & 1 << 1) == 1),
            ((faults & 1 << 2) == 1),
            ((faults & 1 << 3) == 1),
            ((faults & 1 << 4) == 1),
            ((faults & 1 << 5) == 1),
            ((faults & 1 << 6) == 1),
            ((faults & 1 << 7) == 1),
            ((faults & 1 << 8) == 1),
            ((faults & 1 << 9) == 1),
            ((faults & 1 << 10) == 1),
            ((faults & 1 << 11) == 1),
            ((faults & 1 << 12) == 1),
            ((faults & 1 << 13) == 1),
            ((faults & 1 << 14) == 1),
            ((faults & 1 << 15) == 1),
        });
    }
}
