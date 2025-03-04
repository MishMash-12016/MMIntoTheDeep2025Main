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

public class ScoringEndUnitRotatorYAxis extends SubsystemBase {
    public static double yaxisRotaterTransferPos = 0.16;
    public static double yaxisSampleTransferPos = 0.73;
    public static double yaxisScorePos = 0.73;

    private final static MMRobot robotInstance = MMRobot.getInstance();

    public enum ScoringRotatorYAxisState {

        TRANSFER_POSE(()-> yaxisRotaterTransferPos),
        SAMPLE_TRANSFER_POSE(()-> yaxisSampleTransferPos),
        SCORE_POSE(()-> yaxisScorePos);

        public Supplier<Double> position;

        ScoringRotatorYAxisState (Supplier<Double> position) {
            this.position = position;
        }
    }

    CuttleServo servo;
    public ScoringEndUnitRotatorYAxis(){
        servo = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SCORING_YAXIS_ROTATOR);
        servo.setPosition(ScoringRotatorYAxisState.SCORE_POSE.position.get());
    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> {servo.setPosition(newPos);} , this);
    }
    public Command setPosition(ScoringRotatorYAxisState state){
        return new InstantCommand(()-> {
            servo.setPosition(state.position.get());} ,
                this);
    }
}
