
package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.Supplier;

@Config
public class ScoringEndUnitElbow extends SubsystemBase {
    private static final double ElbowMidPose = 0.4;
    private static final double ElbowRestPose = 0.05;
    private static final double ElbowTransferSpecimenPose = 0.09;
    private static final double ElbowTransferSamplePose = 0.1;
    private static final double ElbowScoreSamplePose = 0.63;
    private static final double ElbowInitPose = 0.3;
    private static final double ElbowPrepareSampleTransferPose = 0.7;
    private static final double ElbowScoreSpecimenPose = 0.54;
    private static final double ElbowIntakeFromBackPose = 0.98; /// still needs tuning


    private final static MMRobot robotinstance = MMRobot.getInstance();

    public enum ScoringElbowState {
        MID_POSE(() -> ElbowMidPose),
        REST_POSE(() -> ElbowRestPose),
        TRANSFER_SPECIMEN_POSE(() -> ElbowTransferSpecimenPose),
        PREPARE_SAMPLE_TRANSFER(() -> ElbowPrepareSampleTransferPose),
        TRANSFER_SAMPLE_POSE(() -> ElbowTransferSamplePose),
        SCORE_SAMPLE_POSE(() -> ElbowScoreSamplePose),
        INIT_POSE(() -> ElbowInitPose),
        SCORE_SPECIMEN_POSE(() -> ElbowScoreSpecimenPose),
        INTAKE_FROM_BACK_POSE(() -> ElbowIntakeFromBackPose);


        public final Supplier<Double> position;

        ScoringElbowState(Supplier<Double> position) {
            this.position = position;
        }
    }

    Servo servo;

    public ScoringEndUnitElbow() {
        servo = robotinstance.mmSystems.hardwareMap.get(Servo.class, "scoring rot");
        servo.setPosition(ScoringElbowState.INIT_POSE.position.get());
    }

    public Command setPosition(double newPos) {
        return new InstantCommand(() -> servo.setPosition(newPos),
                this);
    }

    public Command setPosition(ScoringElbowState state) {
        return new InstantCommand(() -> servo.setPosition(state.position.get()),
                this);
    }
}

