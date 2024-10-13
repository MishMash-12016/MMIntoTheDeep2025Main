package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

/**
 * this class represents a subsystem which cannot be categorized under power or position in the current structure.
 * this is smth that i didn't see as relevant enough (read explanation in SubsystemStructure) so i ignored it.
 * but note that in this case (and most of these special uncategorized cases), u would NOT want to make a subsystem like this,
 * due to the same reason as the Shooter subsystem example (the reason is written at the top of the class).
 */
public class ShooterIntake extends SubsystemBase {

    MMRobot mmRobot = MMRobot.getInstance();

    CuttleMotor motor;

    //note that the servo is continues, therefore, not from cuttlefish.
    //read the Configuration class for explanation.
    CRServo crServo;

    public ShooterIntake() {
        motor = mmRobot.mmSystems.controlHub.getMotor(Configuration.SHOOTER_INTAKE);
        crServo = mmRobot.mmSystems.hardwareMap.get(CRServo.class, "" /*Configuration.INTAKE_SERVO*/);
    }

    public void setMotorPower(double power) {
        motor.setPower(power);
    }


    /**
     * this is a WRONG way to use a subsystem.
     * in every subsystem there should be only ONE power/position method.
     * the only exception, is when in every subsystem command, BOTH (or non but that would be dumb) of these methods are used.
     * which means there CAN'T be a command that activates only 1 of these methods,
     * while there's another which activates both or the other (the reason is because they will cancel each-other due to requirements.
     */
    public void setServoPower(double power) {
        crServo.setPower(power);
    }

}
