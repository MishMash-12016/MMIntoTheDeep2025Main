package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class ScoringEndUnitRotatorYAxis extends SubsystemBase {
    private final static MMRobot robotInstance = MMRobot.getInstance();

    public enum ScoringRotatorYAxisState {

        TRANSFER_POSE(1),
        SCORE_SAMPLE_POSE(1),
        SCORE_SPECIMEN_POSE(1),

        INIT_POSE(0.1);

        public final double position;
        ScoringRotatorYAxisState(double position){
            this.position = position;
        }
    }

    CuttleServo servo;
    public ScoringEndUnitRotatorYAxis(){
        servo = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SCORING_YAXIS_ROTATOR);
        servo.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE.position);
    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> {servo.setPosition(newPos);} , this);
    }
    public Command setPosition(ScoringEndUnitRotatorYAxis.ScoringRotatorYAxisState state){
        return new InstantCommand(()-> {
            servo.setPosition(state.position);} ,
                this);
    }
}
