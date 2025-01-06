package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class IntakeClaw extends SubsystemBase {

    CuttleServo clawIntakeServo;
    public enum State {
        OPEN(0.5),
        CLOSE(22);//??
        public double position;

        State(double position) {
            this.position = position;
        }
    }

    // claw close or open
    public IntakeClaw() {
        clawIntakeServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.clawintakeServo);
    }
    public Command openIntakeClaw() {
        return new InstantCommand(() -> clawIntakeServo.setPosition(State.OPEN.position), this);
    }

    public Command closeIntakeClaw() {
        return new InstantCommand(() -> clawIntakeServo.setPosition(State.CLOSE.position), this);
    }


}
