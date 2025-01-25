package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeEndUnitRotator extends SubsystemBase {

    private final static MMRobot robotInstance = MMRobot.getInstance();

    public enum IntakeRotatorState {
        INTAKE_SAMPLE_POSE(0.13),
        HOLD_POSE_SPECIMEN(0.71),
        INTAKE_SPECIMEN_POSE(0.13),
        ROATATE_ANGLE(0.41);
        public double position;

        IntakeRotatorState(double position) {
            this.position = position;
        }
    }

    private final CuttleServo servo;


    public IntakeEndUnitRotator() {
        servo = new CuttleServo(robotInstance.mmSystems.controlHub, 3);
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

    public Command setPositionRUN(double newPos) {
        return new RunCommand(() -> {
            servo.setPosition(newPos);
        },
                this);
    }

    public Command setPosition(IntakeRotatorState state) {
        return new InstantCommand(() -> {
            servo.setPosition(state.position);
        },
                this);
    }

    public Command rotateByButton(BooleanSupplier rotateButton) {
        return new RunCommand(() -> {
            if (rotateButton.getAsBoolean()){
            servo.setPosition(IntakeRotatorState.ROATATE_ANGLE.position);
        } else {
                servo.setPosition(IntakeRotatorState.INTAKE_SAMPLE_POSE.position);
            } }
                , this);
    }

    public Command setPositionByJoystick(DoubleSupplier doubleSupplier) {
        return new RunCommand(() -> {
            servo.setPosition(doubleSupplier.getAsDouble());
        },
                this);
    }

    public Double getTargetPosition() {
        return servo.getPosition();
    }
}
