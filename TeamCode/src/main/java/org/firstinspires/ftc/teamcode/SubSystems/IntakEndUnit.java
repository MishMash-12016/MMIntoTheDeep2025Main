package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class IntakEndUnit extends SubsystemBase {

    CuttleServo clawIntakeServo;
    public enum IntakeClawState {
        OPEN(0.7), CLOSE(1);
        public double position;
        IntakeClawState(double position){
            this.position = position;
        }}

    // claw close or open
    public IntakEndUnit() {
        clawIntakeServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.CLAW_INTAKE_SERVO);
    }
    public Command openIntakeClaw() {
        return new InstantCommand(() -> clawIntakeServo.setPosition(IntakeClawState.OPEN.position), this);
    }

    public Command closeIntakeClaw() {
        return new InstantCommand(() -> clawIntakeServo.setPosition(IntakeClawState.CLOSE.position), this);
    }
}
