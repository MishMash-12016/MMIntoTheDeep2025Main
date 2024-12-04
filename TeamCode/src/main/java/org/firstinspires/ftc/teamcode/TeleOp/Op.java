package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
@TeleOp
public class Op extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();


    public Op() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, 0).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, 1).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, 2).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, 3).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, 4).setPosition(0);

        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 0).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 1).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 2).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 3).setPosition(0);
        new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, 4).setPosition(0);
    }
}
