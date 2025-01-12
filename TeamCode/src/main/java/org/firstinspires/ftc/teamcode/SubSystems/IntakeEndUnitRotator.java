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
        INTAKE_SAMPLE_POSE(0),
        HOLD_POSE_SPECIMEN(1),
        INTAKE_SPECIMEN_POSE(0),
        ROATATE_ANGLE(0.5);
        public double position;

        IntakeRotatorState(double position) {
            this.position = position;
        }
    }

    private final CuttleServo servo;


    public IntakeEndUnitRotator() {
        servo = new CuttleServo(robotInstance.mmSystems.controlHub, Configuration.LINEAR_END_UNIT_ROTATOR);
    }

    public Command setPosition(double newPos) {
        return new InstantCommand(() -> {
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

    public Command rotateByButton(BooleanSupplier bool) {
        return new RunCommand(() -> {
            servo.setPosition(bool.getAsBoolean() ?
                    IntakeRotatorState.ROATATE_ANGLE.position :
                    IntakeRotatorState.INTAKE_SAMPLE_POSE.position);
        },
                this);
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
