
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
    public static double ElbowMidPose = 0.4;
    public static double ElbowRestPose = 0.05;
    public static double ElbowTransferSpecimenPose = 0.13;
    public static double ElbowTransferSamplePose = 0.21;
    public static double ElbowScoreSamplePose = 0.4;
    public static double ElbowInitPose = 0.4;
    public static double ElbowPrepareSampleTransferPose = 0.33;
    public static double ElbowScoreSpecimenPose = 0.58;
    public static double ElbowIntakeFromBackPose = 0.98; /// still needs tuning


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

