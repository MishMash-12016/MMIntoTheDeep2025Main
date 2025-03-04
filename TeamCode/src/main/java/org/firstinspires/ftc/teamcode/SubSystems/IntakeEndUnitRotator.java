package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@Config
public class IntakeEndUnitRotator extends SubsystemBase {
    public static double intakeRotatorSamplePos = 0.31;
    public static double holdPoseSpecimenPos = 0.98;
    public static double intakeSpecimenPos = 0.34;
    public static double rotatorRightAnglePos = 0.57;
    public static double rotatorLeftAnglePos = 0.11;

    private final static MMRobot robotInstance = MMRobot.getInstance();

    public enum IntakeRotatorState {
        INTAKE_SAMPLE_POSE(()-> intakeRotatorSamplePos),
        HOLD_POSE_SPECIMEN(()-> holdPoseSpecimenPos),
        INTAKE_SPECIMEN_POSE(()-> intakeSpecimenPos),
        ROTATE_RIGHT_ANGLE(()-> rotatorRightAnglePos),
        ROTATE_LEFT_ANGLE(()-> rotatorLeftAnglePos);

        public Supplier<Double> position;

        IntakeRotatorState(Supplier<Double> position) {
            this.position = position;
        }
    }

    private final CuttleServo servo;


    public IntakeEndUnitRotator() {
        servo = new CuttleServo(robotInstance.mmSystems.controlHub, Configuration.LINEAR_END_UNIT_ROTATOR);
        servo.setPosition(IntakeRotatorState.INTAKE_SAMPLE_POSE.position.get());
    }

    public Command setPosition(double newPos) {
        return new InstantCommand(() -> {
            servo.setPosition(newPos);
        },
                this);
    }
    public void setPositionVoid(double newPos) {
        servo.setPosition(newPos);
    }

    public Command setPosition(IntakeRotatorState state) {
        return new InstantCommand(() -> {
            servo.setPosition(state.position.get());
        },
                this);
    }

    public Command rotateByButton(BooleanSupplier rotateLeftButton,BooleanSupplier rotateRightButton) {
        return new RunCommand(() -> {
            if (rotateLeftButton.getAsBoolean()){
                servo.setPosition(IntakeRotatorState.ROTATE_LEFT_ANGLE.position.get());
            }else if (rotateRightButton.getAsBoolean()) {
                servo.setPosition(IntakeRotatorState.ROTATE_RIGHT_ANGLE.position.get());
            }
            else {
                servo.setPosition(IntakeRotatorState.INTAKE_SAMPLE_POSE.position.get());
            } }
                , this);
    }
}