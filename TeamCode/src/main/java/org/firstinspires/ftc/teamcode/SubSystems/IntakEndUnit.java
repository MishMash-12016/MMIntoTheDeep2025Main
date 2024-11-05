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
    CuttleServo intakeArm;
    CRServo rollerServo;
    public final double intakePower = 0.5;

    public IntakEndUnit() {
        rollerServo = MMRobot.getInstance().mmSystems.hardwareMap.crservo.get(Configuration.rollerServo);
    }
    public Command setIntakePower() {
        return new RunCommand(() -> rollerServo.setPower(intakePower), this);
    }

    public Command stopIntakePower() {
        return new InstantCommand(() -> rollerServo.setPower(0), this);
    }


}
