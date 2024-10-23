package org.firstinspires.ftc.teamcode.TeleOps;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
@TeleOp
@Config
public class NormalTeleOpTest extends MMOpMode {

    public NormalTeleOpTest(){
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initElevator();
        MMRobot.getInstance().mmSystems.initBumper();
    }


    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();


        MMRobot.getInstance().mmSystems.elevator.setPower((double)(gamepad1.left_trigger - gamepad1.right_trigger));
        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();

    }
}