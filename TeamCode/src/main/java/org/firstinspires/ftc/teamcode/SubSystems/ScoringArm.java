package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class ScoringArm extends SubsystemBase {
    private final CuttleServo servoLeft;
    private final CuttleServo servoRight;
    public static double transferHold = 0.1;
    public static double midPose = 0.43;



    public final static double  scoreSpecimen= 0.5;
    public final static double scoreSampleHigh = 0.5;
    public final static double scoreSampleLow = 0.6;

    public ScoringArm() {
        servoLeft = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SERVO_LEFT_SCORING_ARM);
        servoRight = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SERVO_RIGHT_SCORING_ARM);
    }

    //Tell arm to get to position
    public Command setleftPosition(double newPos) {
        return new InstantCommand(()-> {
            servoLeft.setPosition(newPos);},
            //servoRight.setPosition(newPos);} ,
                this);
    }
    public Command setPosition(double newPos) {
        return new InstantCommand(()-> {
            servoLeft.setPosition(newPos);
                servoRight.setPosition(1-newPos);} ,
                this);
    }
    public Command setrightPosition(double newPos) {
        return new InstantCommand(()-> {
            servoRight.setPosition(newPos);} ,
                this);
    }
}