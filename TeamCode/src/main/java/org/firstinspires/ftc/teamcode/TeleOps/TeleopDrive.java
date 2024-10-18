package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.ElevatorBackTo_0;
import org.firstinspires.ftc.teamcode.CommandGroup.Intake;
import org.firstinspires.ftc.teamcode.CommandGroup.OutTakeRoller;
import org.firstinspires.ftc.teamcode.CommandGroup.Scoring;
import org.firstinspires.ftc.teamcode.Commands.ClawSetState;
import org.firstinspires.ftc.teamcode.Commands.IntakeArmSetState;
import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ScoringArmSetState;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands.ResetFieldOrientedCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class TeleopDrive extends MMTeleOp {

    MMRobot robot = MMRobot.getInstance();

    public TeleopDrive() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.initDriveTrain();
        MMRobot.getInstance().mmSystems.initIntake();
        MMRobot.getInstance().mmSystems.initLinearIntake();
        MMRobot.getInstance().mmSystems.initElevator();
        MMRobot.getInstance().mmSystems.initIntakeArm();
        MMRobot.getInstance().mmSystems.initScoringArm();
        MMRobot.getInstance().mmSystems.initClaw();


        addRunnableOnInit(
                () -> MMRobot.getInstance().mmSystems.linearIntake.setPosition(0)
        );
        addCommandsOnRun(
                new IntakeArmSetState(IntakeArm.Position.IN),
                new ClawSetState(MMRobot.getInstance().mmSystems.claw,Claw.State.OPEN),
                new ScoringArmSetState(MMRobot.getInstance().mmSystems.scoringArm, ScoringArm.Position.IN)
        );



        Trigger leftTriggerCondition = new Trigger(
            () -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05
        );
        leftTriggerCondition.whenActive(
                new Intake(leftTriggerCondition)
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new OutTakeRoller()
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ElevatorBackTo_0()
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new Scoring(
                        MMRobot.getInstance().mmSystems.elevator,
                        MMRobot.getInstance().mmSystems.scoringArm,
                        MMRobot.getInstance().mmSystems.claw,
                        MMRobot.getInstance().mmSystems.elevator.LOW_BASCET
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new Scoring(
                        MMRobot.getInstance().mmSystems.elevator,
                        MMRobot.getInstance().mmSystems.scoringArm,
                        MMRobot.getInstance().mmSystems.claw,
                        MMRobot.getInstance().mmSystems.elevator.HIGH_BASKET
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ResetFieldOrientedCommand()
        );


    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("imu: ", MMRobot.getInstance().mmSystems.imu.getYawInDegrees());
        telemetry.update();

    }

}