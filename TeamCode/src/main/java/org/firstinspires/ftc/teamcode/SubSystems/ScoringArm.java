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

    public enum ScoringArmState {
        TRANSFER_POSE(0.83),
        MID_POSE(0.43),
        SCORE_SPECIMEN(0.48),
        TRANSFER_SPECIMEN_POSE(0.82),
        SCORE_SAMPLE(0.3);
        public double position;
        ScoringArmState(double position){
            this.position = position;
        }}

    public ScoringArm() {
        servoLeft = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SERVO_LEFT_SCORING_ARM);
        servoRight = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SERVO_RIGHT_SCORING_ARM);
        servoLeft.setPosition(ScoringArmState.MID_POSE.position);
        servoRight.setPosition(-ScoringArmState.MID_POSE.position);
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
                servoRight.setPosition(newPos);} ,
                this);
    }


    public Command setPosition(ScoringArmState state) {
        return new InstantCommand(()-> {
            servoLeft.setPosition(state.position);
            servoRight.setPosition(1-state.position);} ,
                this);
    }
    public Command setrightPosition(double newPos) {
        return new InstantCommand(()-> {
            servoRight.setPosition(newPos);} ,
                this);
    }
    public double getPosition(){
        return servoRight.getPosition();
    }
}