package org.firstinspires.ftc.teamcode.TeleOps;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CommandGroup.Scoring;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.OpModeType;

@TeleOp
public class NormalTeleOpTest extends MMTeleOp {

    public NormalTeleOpTest(){
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.initDriveTrain();
//        MMRobot.getInstance().mmSystems.initIntake();
//        MMRobot.getInstance().mmSystems.initLinearIntake();
//        MMRobot.getInstance().mmSystems.initElevator();
//        MMRobot.getInstance().mmSystems.initIntakeArm();
//        MMRobot.getInstance().mmSystems.initScoringArm();
//        MMRobot.getInstance().mmSystems.initClaw();



//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new MMPIDCommand(MMRobot.getInstance().mmSystems.elevator,20)
//        );
//
//
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                new Scoring(
//                        MMRobot.getInstance().mmSystems.elevator,
//                        MMRobot.getInstance().mmSystems.scoringArm,
//                        MMRobot.getInstance().mmSystems.claw,
//                        MMRobot.getInstance().mmSystems.elevator.LOW_BASCET
//                )
//        );


    }

    @Override
    public void run() {
        super.run();
//        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();


    }
}