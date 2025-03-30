package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class Hook extends SubsystemBase {
    CuttleServo rightHookServo;
    CuttleServo leftHookServo;
    public enum HookState {
        OPEN(0.5), CLOSE(0);
        public final double position;
        HookState(double position){
            this.position = position;
        }}

    public Hook() {
        rightHookServo = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.RIGHT_HOOK);
        leftHookServo= new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.LEFT_HOOK);

        rightHookServo.setPosition(HookState.CLOSE.position);
        leftHookServo.setPosition(1-HookState.CLOSE.position);
    }
    public Command OpenHook(){
        return setPosition(HookState.OPEN.position);
    }
    public Command CloseHook(){
        return setPosition(HookState.CLOSE.position);
    }

    public Command setPosition(double pose) {
        return new InstantCommand(() -> {
            leftHookServo.setPosition(1-pose);
            rightHookServo.setPosition(pose);
        },
                this);
    }
    public Command setPosition(HookState state){
        return setPosition(state.position);
    }
}

