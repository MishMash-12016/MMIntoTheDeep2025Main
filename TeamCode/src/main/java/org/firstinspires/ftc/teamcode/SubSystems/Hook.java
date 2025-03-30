package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class Hook extends SubsystemBase {
    CuttleServo wisherServo;
    public enum WisherState {
        OUT_POSE(1), IN_POSE(0);
        public final double position;
        WisherState(double position){
            this.position = position;
        }}

    public Hook() {
        wisherServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.WISHER);
        wisherServo.setPosition(WisherState.IN_POSE.position);
    }
    public Command HookOut() {
        return new InstantCommand(() -> wisherServo.setPosition(WisherState.OUT_POSE.position), this);
    }
    public Command HookIn() {
        return new InstantCommand(() -> wisherServo.setPosition(WisherState.IN_POSE.position), this);
    }

    public Command setPosition(double pose) {
        return new InstantCommand(() -> wisherServo.setPosition(pose), this);
    }
    public Command setPosition(WisherState state){
        return setPosition(state.position);
    }
}

