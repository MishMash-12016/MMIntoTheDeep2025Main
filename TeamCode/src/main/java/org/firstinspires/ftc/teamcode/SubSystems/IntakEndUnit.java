package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class IntakEndUnit extends SubsystemBase {

    CuttleServo clawintakeServo;
    public final double open = 0.5;
    public final double close = 22;

    // claw close or open
    public IntakEndUnit() {
        clawintakeServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.clawintakeServo);
    }
    public Command openIntakeClaw() {
        return new InstantCommand(() -> clawintakeServo.setPosition(open), this);
    }

    public Command closeIntakeClaw() {
        return new InstantCommand(() -> clawintakeServo.setPosition(close), this);
    }


}
