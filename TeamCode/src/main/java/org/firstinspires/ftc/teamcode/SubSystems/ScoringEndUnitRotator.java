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
    public static double rotatorSpecimenTransferPose = 0.14;
    public static double rotatorSampleTransferPose = 0.81;
    public static double rotatorScoringSpecimenPose = 0.14;
    public static double rotatorScoringSamplePose = 0.14;
    public static double rotatorSpecimenSideScore = 0.27;


    private final static MMRobot robotInstance = MMRobot.getInstance();

    public enum ScoringRotatorState {

        SPECIMEN_TRANSFER_POSE(()-> rotatorSpecimenTransferPose),
        SAMPLE_TRANSFER_POSE(()-> rotatorSampleTransferPose),
        SCORING_SPECIMEN_POSE(()-> rotatorScoringSpecimenPose),
        SCORING_SAMPLE_POSE(() -> rotatorScoringSamplePose),
        SCORING_SPECIMEN_SIDE_POSE(() -> rotatorSpecimenSideScore);

        public final Supplier<Double> position;

        ScoringRotatorState(Supplier<Double> position) {
            this.position = position;
        }
    }

    CuttleServo servo;
    public ScoringEndUnitRotator(){
        servo = new CuttleServo(robotInstance.mmSystems.expansionHub, Configuration.SCORING_ROTATOR);
        servo.setPosition(ScoringRotatorState.SPECIMEN_TRANSFER_POSE.position.get());
    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> servo.setPosition(newPos), this);
    }
    public Command setPosition(ScoringRotatorState state){
        return new InstantCommand(()-> servo.setPosition(state.position.get()),
                this);
    }
}
