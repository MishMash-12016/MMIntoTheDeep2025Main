package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class Wisher extends SubsystemBase {
    CuttleServo wisherServo;
    public enum WisherState {
        OUT(1), IN(0);
        public double position;
        WisherState(double position){
            this.position = position;
        }}

    public Wisher() {
        wisherServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.WISHER);
        wisherServo.setPosition(WisherState.IN.position);
    }
    public Command WisherOut() {
        return new InstantCommand(() -> wisherServo.setPosition(WisherState.OUT.position), this);
    }
    public Command WisherIn() {
        return new InstantCommand(() -> wisherServo.setPosition(WisherState.IN.position), this);
    }
    public Command Wish() {
        return new SequentialCommandGroup(
                WisherOut(),
                new WaitCommand(50),
                WisherIn()
        );
    }
    public Command setPosition(double pose) {
        return new InstantCommand(() -> wisherServo.setPosition(pose), this);
    }
    public Command setPosition(WisherState state){
        return new InstantCommand(()-> {
            wisherServo.setPosition(state.position);} ,
                this);
    }


}

