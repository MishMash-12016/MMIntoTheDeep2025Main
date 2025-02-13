package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MMRobot;

public class ScoringArm extends SubsystemBase {
    public enum ScoringArmState {
        TRANSFER_POSE(0.715),
        PARK_AUTO(0.3),
        INIT_POSE(0.72),
        PREPARE_TRANSFER(0.63),
        MID_POSE(0.43),
        SCORE_SPECIMEN(0.34),
        TRANSFER_SPECIMEN_POSE(0.72),
        SCORE_SAMPLE(0.26),
        PREPARE_SCORE_SAMPLE(0.29);
        public double position;
        ScoringArmState(double position){
            this.position = position;
        }}
    Servo servoLeft;
    Servo servoRight;

    public ScoringArm() {
         servoLeft = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "L outake arm ");
        servoRight = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "R outake arm");
        servoLeft.setPosition(ScoringArmState.INIT_POSE.position);
        servoRight.setPosition(1-ScoringArmState.INIT_POSE.position);
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

    public void CutPower(){
        servoLeft.getController().pwmDisable();
        servoRight.getController().pwmDisable();
    }

    public double getPosition(){
        return servoRight.getPosition();
    }

    public void CutPower() {
        servoLeft.getController().pwmDisable();
        servoRight.getController().pwmDisable();
    }
}