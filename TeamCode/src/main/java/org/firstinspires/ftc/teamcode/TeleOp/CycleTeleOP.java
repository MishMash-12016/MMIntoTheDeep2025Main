package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class CycleTeleOP extends MMOpMode {

    public CycleTeleOP() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.initRobotSystems();
        MMRobot.getInstance().mmSystems.initDriveTrain();

        //Buttons:
        //Gamepad 1:
        Trigger intakeCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05);
        Trigger intakeDoneCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.A));
        Trigger changeIntakeRotator = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.B));
        Trigger scoreSampleCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER));
        Trigger restYawCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        Trigger clawscoring = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05);

        //Gamepad 2:
        Trigger prepareHighSampleCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.DPAD_UP));
        Trigger prepareLowSampleCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.DPAD_DOWN));
        Trigger foldSystemCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.B));
        Trigger ejectCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.Y));
        Trigger elavtaordown = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05);
        Trigger elavatorup = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05);
        Trigger resetelavtor = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.A));
        Trigger intakespeciman = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        Trigger goDownElevator = new Trigger(()-> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.X));
        //Buttons:
        //gamepad 1: intake,intake done,rotate,score,drive,rest imu
        intakeCondition.whileActiveOnce(RobotCommands.IntakeCommand(() -> gamepad1.right_trigger) //right trigger
        );
        intakeDoneCondition.whenActive(RobotCommands.IntakeDoneCommand()); //a
        changeIntakeRotator.whenActive(MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.getTargetPosition() == IntakeEndUnitRotator.rotateangle ? IntakeEndUnitRotator.intakeSamplePose : IntakeEndUnitRotator.rotateangle)); //B
        scoreSampleCondition.whenActive(RobotCommands.ScoreSample()); //y
        restYawCondition.whenActive(() -> MMRobot.getInstance().mmSystems.imu.resetYaw()); //right bumper
        clawscoring.whenActive(
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
        );

        //gamepad 2: prepare high,prepare low,fold,eject
        prepareHighSampleCondition.whenActive(RobotCommands.PrepareHighSample()); // dpad up
        prepareLowSampleCondition.whenActive(RobotCommands.PrepareLowSample()); //dpad down
        foldSystemCondition.whenActive(RobotCommands.FoldSystems()); // B
        ejectCondition.whenActive(RobotCommands.EjectSampleCommand()); // Y
        elavtaordown.whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(-0.6)); //left trigger
        elavatorup.whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.6)); //right trigger
        resetelavtor.whenActive(() -> MMRobot.getInstance().mmSystems.elevator.resetTicks());
        elavtaordown.whenInactive(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0));
        elavatorup.whenInactive(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0));
        intakespeciman.whenActive(
                RobotCommands.prepareSpecimenIntake()
        );
        goDownElevator.whenActive(MMRobot.getInstance().mmSystems.elevator.moveToPose(0.05));


    }


    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("height", MMRobot.getInstance().mmSystems.elevator.getHeight());
        telemetry.addData("right trigger", gamepad1.right_trigger);
        telemetry.update();
    }
}
