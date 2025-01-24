package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;


public class ScoringClawEndUnit extends SubsystemBase {
    Servo clawScoringServo;
    public enum ScoringClawState {
        OPEN(0.4), CLOSE(0),BARELY_OPEN(0.07);
        public double position;
        ScoringClawState(double position){
            this.position = position;
        }}


    public ScoringClawEndUnit() {

        clawScoringServo = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "Outake claw");
    }

    public Command openScoringClaw() {
        return new InstantCommand(() -> {
            clawScoringServo.setPosition(ScoringClawState.OPEN.position);}, this);
    }


    public Command setPosition(double newPos){
        return new InstantCommand(()-> {
            clawScoringServo.setPosition(newPos);} ,
                this);
    }
    public Command setPosition(ScoringClawState scoringClawState){
        return new InstantCommand(()-> {
            clawScoringServo.setPosition(scoringClawState.position);} ,
                this);
    }

    public Command closeScoringClaw() {
        return new InstantCommand(() -> {
                clawScoringServo.setPosition(ScoringClawState.CLOSE.position);}, this);
    }


}
