package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.roboctopi.cuttlefish.utils.Direction;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class Bumper extends SubsystemBase {

    CuttleMotor motorRight;
    CuttleMotor motorLeft;

    public Bumper(){
        motorRight = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.BUMPER_RIGHT);
        motorLeft = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.BUMPER_LEFT);

        motorRight.setDirection(Direction.REVERSE);
    }

    public void setPower(double power){
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }


}
