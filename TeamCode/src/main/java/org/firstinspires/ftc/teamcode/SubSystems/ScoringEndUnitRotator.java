package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.Supplier;

@Config

public class ScoringEndUnitRotator extends SubsystemBase {
    public static double RotaterTransferPos = 0.18;
    public static double RotatorSampleTransferPos = 0.73;
    public static double rotaterScorePos = 0.73;


    private final static MMRobot robotInstance = MMRobot.getInstance();

    public enum ScoringRotatorState {

        TRANSFER_POSE(()-> RotaterTransferPos),
        SAMPLE_TRANSFER_POSE(()-> RotatorSampleTransferPos),
        SCORE_POSE(()-> rotaterScorePos);

        public Supplier<Double> position;

        ScoringRotatorState(Supplier<Double> position) {
            this.position = position;
        }
    }

    CuttleServo servo;
    public ScoringEndUnitRotator(){
        servo = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SCORING_YAXIS_ROTATOR);
        servo.setPosition(ScoringRotatorState.SCORE_POSE.position.get());
    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> {servo.setPosition(newPos);} , this);
    }
    public Command setPosition(ScoringRotatorState state){
        return new InstantCommand(()-> {
            servo.setPosition(state.position.get());} ,
                this);
    }
}
