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
    public static double prepareSampleScorePose = 0.42;
    public static double ElbowScoreSamplePose = 0.78;
    public static double ElbowInitPose = 0.32;
    public static double ElbowTransferSamplePose = 0.1;
    public static double ElbowPrepareSampleTransferPose = 0.09;
    public static double ElbowScoreSpecimenPose = 0.73;
    public static double ElbowAfterScoreSpecimenPose = 0.64;
    public static double ElbowIntakeFromFrontPose = 0.4;
    public static double scoringElbowScoreFromFrontPose = 0.53;
    public static double ElbowAfterScoreFromFront = 0.35;
    public static double ElbowPark = 0.38;
    public static double est = 0;


    private final static MMRobot robotinstance = MMRobot.getInstance();

    public enum ScoringElbowState {
        ELBOW_SCORE_FROM_FRONT_POSE(()-> scoringElbowScoreFromFrontPose),
        ELBOW_AFTER_SCORE_FROM_FRONT(() -> ElbowAfterScoreFromFront),
        PREPARE_SAMPLE_TRANSFER(() -> ElbowPrepareSampleTransferPose),
        TRANSFER_SAMPLE_POSE(() -> ElbowTransferSamplePose),
        SCORE_SAMPLE_POSE(() -> ElbowScoreSamplePose),
        INIT_POSE(() -> ElbowInitPose),
        SCORE_SPECIMEN_POSE(() -> ElbowScoreSpecimenPose),
        AFTER_SCORE_SPECIMEN_POSE(() -> ElbowAfterScoreSpecimenPose),
        INTAKE_FROM_FRONT_POSE(() -> ElbowIntakeFromFrontPose),
        PARK(() -> ElbowPark),
        PREPARE_SAMPLE_SCORE(() -> prepareSampleScorePose);

        public final Supplier<Double> position;

        ScoringElbowState(Supplier<Double> position) {
            this.position = position;
        }
    }

    Servo servo;

    public ScoringEndUnitElbow() {
        servo = robotinstance.mmSystems.hardwareMap.get(Servo.class, "scoring elbow");
        servo.setPosition(ScoringElbowState.INIT_POSE.position.get());
        est = ScoringElbowState.INIT_POSE.position.get();
    }

    public ScoringEndUnitElbow(Boolean Void) {
        servo = robotinstance.mmSystems.hardwareMap.get(Servo.class, "scoring elbow");
        servo.setPosition(ScoringElbowState.PARK.position.get());
        est = ScoringElbowState.PARK.position.get();
    }

    public ScoringEndUnitElbow(int dontMode) {
        servo = robotinstance.mmSystems.hardwareMap.get(Servo.class, "scoring elbow");
    }

    public Command setPosition(double newPos) {
        return new InstantCommand(() -> {
            servo.setPosition(newPos);
            est = newPos;
        },
                this);
    }

    public Command setPosition(ScoringElbowState state) {
        return new InstantCommand(() -> {
            servo.setPosition(state.position.get());
            est = state.position.get();
        },
                this);
    }
    public Command setPositionWithoutRequirments(ScoringElbowState state) {
        return new InstantCommand(() -> {
            servo.setPosition(state.position.get());
            est = state.position.get();
        });
    }
    public static double getPosition() {
        return est;
    }
}