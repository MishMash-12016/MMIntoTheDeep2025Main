package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class IntakeArm extends SubsystemBase {
    CuttleServo intakeArm;

    public final double up = 0.5;
    public final double down = -1.5;

    public IntakeArm() {
        intakeArm = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.intakeArm);
    }

    //tell servo intake to get to down position
    public Command intakeUp() {
        return new InstantCommand(() -> intakeArm.setPosition(up), this);
    }

    public Command intakeDown() {
        return new InstantCommand(() -> intakeArm.setPosition(down), this);
    }
}
