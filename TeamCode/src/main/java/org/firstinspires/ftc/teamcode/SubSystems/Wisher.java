package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class Wisher extends SubsystemBase {
    CuttleServo wisherServo;
    public enum WisherState {
        OPEN(0.1), CLOSE(1);
        public double position;
        WisherState(double position){
            this.position = position;
        }}

    public Wisher() {
        wisherServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.WISHER);
        wisherServo.setPosition(WisherState.OPEN.position);
    }
    public Command openWisher() {
        return new InstantCommand(() -> wisherServo.setPosition(WisherState.OPEN.position), this);
    }
    public Command closeWisher() {
        return new InstantCommand(() -> wisherServo.setPosition(WisherState.CLOSE.position), this);
    }
    public Command Wish() {
        return new SequentialCommandGroup(
                openWisher(),
                new WaitCommand(50),
                closeWisher()
        );
    }
    public Command setPosition(double pose) {
        return new InstantCommand(() -> wisherServo.setPosition(pose), this);
    }
    public Command setPosition(Wisher.WisherState state){
        return new InstantCommand(()-> {
            wisherServo.setPosition(state.position);} ,
                this);
    }


}

