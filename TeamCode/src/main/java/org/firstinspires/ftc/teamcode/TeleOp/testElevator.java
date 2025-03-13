package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class testElevator extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    boolean Specimenintake = true;


    public testElevator() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;

        //mmSystems.elevator.setDefaultCommand(new RunCommand(()->{},mmSystems.elevator));


        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORING_SPECIMEN_SIDE_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SPECIMEN_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_SPECIMEN)
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                IntakeSpecimenCommand.IntakeFromFront()
        );
    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
        mmSystems.driveTrain.updateTelemetry();

        telemetry.addData("targertpose", mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//        telemetry.addData("ticks - ", mmSystems.elevator.getTicks());
//        telemetry.addData("height", mmSystems.elevator.getHeight());
//        telemetry.addData("power", mmSystems.elevator.getPower());
        telemetry.update();

    }
}