package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class TuningTest extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    boolean Specimenintake = true;

    private static double posRot = 0.55;
    private static double posElbow = 0.59+0.035;
    private static double posArm = 0.71;
    private static final double changeBy = 0.005;
    public TuningTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;

        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORING_SPECIMEN_SIDE_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORING_SPECIMEN_SIDE_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_SPECIMEN_SIDE_POSE)
                )
        );
        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whenActive(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORING_SPECIMEN_SIDE_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.AFTER_SPECIMEN_SCORE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.AFTER_SCORE_POSE)));



        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posRot += changeBy;
                            ScoringEndUnitRotator.rotatorSpecimenSideScore = posRot;
                        })
                ).andThen(
                        new ParallelCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_SPECIMEN_SIDE_POSE)
                        )
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posRot -= changeBy;
                            ScoringEndUnitRotator.rotatorSpecimenSideScore = posRot;
                        })
                ).andThen(
                        new ParallelCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_SPECIMEN_SIDE_POSE)
                        )
                )
        );



        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posElbow += changeBy;
                            ScoringEndUnitElbow.elbowSpecimenSideScore = posElbow;
                        })
                ).andThen(
                        new ParallelCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_SPECIMEN_SIDE_POSE)
                        )
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posElbow -= changeBy;
                            ScoringEndUnitElbow.elbowSpecimenSideScore = posElbow;
                        })
                ).andThen(
                        new ParallelCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_SPECIMEN_SIDE_POSE)
                        )
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posArm += changeBy;
                            ScoringArm.scoringArmSpecimenSideScore = posArm;
                        })
                ).andThen(
                        new ParallelCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_SPECIMEN_SIDE_POSE)
                        )
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posArm -= changeBy;
                            ScoringArm.scoringArmSpecimenSideScore = posArm;
                        })
                ).andThen(
                        new ParallelCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORING_SPECIMEN_SIDE_POSE),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_SPECIMEN_SIDE_POSE)
                        )
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
        );



        /*
        DPAD_UP + rot
        DPAD_DOWN - rot
        DPAD_LEFT + elbow
        DPAD_RIGHT - elbow
        Y + arm
        A - arm
        X: action
         */
    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();

        telemetry.addData("rot", posRot);
        telemetry.addData("elbow", posElbow);
        telemetry.addData("arm", posArm);

        telemetry.update();

    }
}