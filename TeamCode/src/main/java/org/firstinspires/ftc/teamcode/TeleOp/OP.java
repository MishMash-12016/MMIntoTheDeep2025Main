package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class OP extends MMOpMode {

    public OP() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub,0).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub,1).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub,2).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub,3).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub,4).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub,5).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub,0).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub,1).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub,2).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub,3).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub,4).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub,5).setPosition(0);

    }
}
