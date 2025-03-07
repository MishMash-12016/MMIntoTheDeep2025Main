
package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

import java.util.function.Supplier;

@Config
public class ScoringEndUnitElbow extends SubsystemBase {
    public static double restPose = 0.73;
    public static double transferSpecimenPose = 0.4;
    public static double transferSamplePose = 0.45;
    public static double scoreSamplePose = 0.61;
    public static double initPose = 0.7;
    public static double prepareSampleTransferPose = 0.7;
    public static double scoreSpecimenPose = 0.98;
    public static double intakeFromBackPose = 0.98; /// still needs tuning


    private final static MMRobot robotinstance = MMRobot.getInstance();

    public enum ScoringElbowState {
        REST_POSE(() -> restPose),
        TRANSFER_SPECIMEN_POSE(() -> transferSpecimenPose),
        PREPARE_SAMPLE_TRANSFER(() -> prepareSampleTransferPose),
        TRANSFER_SAMPLE_POSE(() -> transferSamplePose),
        SCORE_SAMPLE_POSE(() -> scoreSamplePose),
        INIT_POSE(() -> initPose),
        SCORE_SPECIMEN_POSE(() -> scoreSpecimenPose),
        INTAKE_FROM_BACK_POSE(() -> intakeFromBackPose);


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

