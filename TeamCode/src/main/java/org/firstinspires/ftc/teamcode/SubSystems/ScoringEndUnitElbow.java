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
    public static double ElbowMidPose = 0.4 + 0.035;
    public static double ElbowRestPose = 0.05 + 0.035;
    public static double ElbowTransferSpecimenPose = 0.165;
    public static double ElbowTransferSamplePose = 0.19 + 0.035;
    public static double ElbowScoreSamplePose = 0.48 + 0.035;
    public static double ElbowInitPose = 0.3;
    public static double ElbowPrepareSampleTransferPose = 0.27 + 0.035;
    public static double ElbowScoreSpecimenPose = 0.59 + 0.035;
    public static double ElbowIntakeFromFrontPose = 0.26 + 0.035;
    /// still needs tuning
    public static double prepareSampleScorePose = 0.38 + 0.035;
    public static double afterSpecimenScore = 0.345;

    public static double elbowSpecimenSideScore = 0.25;
    public static double scoringElbowAfterScoreFromFrontSpecimenPose = 0.685;


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
        INTAKE_FROM_FRONT_POSE(() -> ElbowIntakeFromFrontPose),
        PREPARE_SAMPLE_SCORE(() -> prepareSampleScorePose),

        SCORING_SPECIMEN_SIDE_POSE(() -> elbowSpecimenSideScore),
        AFTER_SPECIMEN_SCORE(() -> afterSpecimenScore),
        AFTER_SCORING_FRONT_SPECIMEN_POSE(() -> scoringElbowAfterScoreFromFrontSpecimenPose);


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