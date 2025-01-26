package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class ScoringArm extends SubsystemBase {
    public enum ScoringArmState {
        TRANSFER_POSE(0.74),
        PARK_AUTO(0.3),
        INIT_POSE(0.73),
        PREPARE_TRANSFER(0.63),
        MID_POSE(0.43),
        SCORE_SPECIMEN(0.35),
        TRANSFER_SPECIMEN_POSE(0.71),
        SCORE_SAMPLE(0.32);
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
    public double getPosition(){
        return servoRight.getPosition();
    }
}