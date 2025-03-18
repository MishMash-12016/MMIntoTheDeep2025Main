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
    public static double ElbowMidPose = 0.715;
    public static double ElbowRestPose = 0.365;
    public static double ElbowTransferSpecimenPose = 0.445;
    public static double ElbowTransferSamplePose =0.495;
    public static double ElbowScoreSamplePose = 0.785;
    public static double ElbowInitPose = 0.56;
    public static double ElbowPrepareSampleTransferPose = 0.575;
    public static double ElbowScoreSpecimenPose = 0.765;
    public static double ElbowIntakeFromFrontPose = 0.56; //0.505
    public static double scoringElbowMidToFront = 0.45;
    public static double prepareSampleScorePose = 0.685;
    public static double afterSpecimenScore = 0.705;
    public static double scoringElbowScoreFromFrontSpecimenPose = 0.645;
    public static double scoringElbowAfterScoreFromFrontSpecimenPose = 0.845;
    public static double elbowSpecimenSideScore = 0.425;
    public static double scoringElbowAfterScoreFromSideSpecimenPose = 0.6;
    public static double ElbowTelOpInitPose =0.78;


    private final static MMRobot robotinstance = MMRobot.getInstance();

    public enum ScoringElbowState {
        ELBOW_SCORE_FROM_FRONT_POSE(()-> scoringElbowScoreFromFrontSpecimenPose),
        MID_POSE(() -> ElbowMidPose),
        REST_POSE(() -> ElbowRestPose),
        TRANSFER_SPECIMEN_POSE(() -> ElbowTransferSpecimenPose),
        PREPARE_SAMPLE_TRANSFER(() -> ElbowPrepareSampleTransferPose),
        TRANSFER_SAMPLE_POSE(() -> ElbowTransferSamplePose),
        SCORE_SAMPLE_POSE(() -> ElbowScoreSamplePose),
        INIT_POSE(() -> ElbowInitPose),
        TELOP_INIT_POSE(() -> ElbowTelOpInitPose),
        SCORE_SPECIMEN_POSE(() -> ElbowScoreSpecimenPose),
        INTAKE_FROM_FRONT_POSE(() -> ElbowIntakeFromFrontPose),
        PREPARE_SAMPLE_SCORE(() -> prepareSampleScorePose),
        SCORING_SPECIMEN_SIDE_POSE(() -> elbowSpecimenSideScore),
        AFTER_SPECIMEN_SCORE(() -> afterSpecimenScore),
        AFTER_SCORING_FRONT_SPECIMEN_POSE(() -> scoringElbowAfterScoreFromFrontSpecimenPose),

        AFTER_SCORING_SIDE_SPECIMEN_POSE(() -> scoringElbowAfterScoreFromSideSpecimenPose),
        MID_TO_FRONT(() -> scoringElbowMidToFront);


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
    public ScoringEndUnitElbow(boolean Void) {
        servo = robotinstance.mmSystems.hardwareMap.get(Servo.class, "scoring rot");
        servo.setPosition(ScoringElbowState.TELOP_INIT_POSE.position.get());
    }

    public Command setPosition(double newPos) {
        return new InstantCommand(() -> servo.setPosition(newPos),
                this);
    }

    public Command setPosition(ScoringElbowState state) {
        return new InstantCommand(() -> servo.setPosition(state.position.get()),
                this);
    }

    public double getPosition() {
        return servo.getPosition();
    }
}