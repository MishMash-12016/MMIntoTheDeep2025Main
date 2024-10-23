package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class
RollerIntake extends SubsystemBase{

    MMRobot mmRobot = MMRobot.getInstance();

    CRServo servo;
    public boolean isRoll = false;
    public final static double ON = 1;
    public final static double OFF = 0;

    public RollerIntake() {
        servo = MMRobot.getInstance().mmSystems.hardwareMap.crservo.get(Configuration.ROLLER_INTAKE);

    }

    public void setPower(double power){servo.setPower(-power);}

    public void setIsRoll(Boolean isRoll){this.isRoll = isRoll;}

}
