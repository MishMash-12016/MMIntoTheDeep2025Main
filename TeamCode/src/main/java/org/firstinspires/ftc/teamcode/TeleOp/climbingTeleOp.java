package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class climbingTeleOp extends MMOpMode {
    public climbingTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initRobotSystems();
    }

    @Override
    public void run(){
        super.run();
        MMRobot.getInstance().mmSystems.elevator.setPower(MMRobot.getInstance().mmSystems.gamepadEx1.getLeftY());
    }
}
