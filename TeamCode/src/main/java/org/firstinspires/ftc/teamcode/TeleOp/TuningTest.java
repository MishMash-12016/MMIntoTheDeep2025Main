package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.ScoringArm_SpeedControll;
import org.firstinspires.ftc.teamcode.CommandGroup.touchSensors;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.AbstractSet;
import java.util.HashMap;

@TeleOp
@Config
public class TuningTest extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;

    private static double posElbow = ScoringEndUnitElbow.ScoringElbowState.INIT_POSE.position.get();
    private static double posArm = ScoringArm.ScoringArmState.INIT_POSE.position.get();
    private static double posIntakeArm = IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE.position.get();
    public static double time = 900;

    HashMap<String, Double> poses = new HashMap<>();


    private static final double changeBy = 0.01;
    public TuningTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        poses.put("elbow", posElbow);
        poses.put("ScoringArm", posArm);
        poses.put("intakeArm", posIntakeArm);


        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;

        robotInstance.mmSystems.initRobotSystemsTeleOp(this);
        robotInstance.mmSystems.initDriveTrain();


///intake arm
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            posIntakeArm += changeBy;
//                            IntakeArm.intakeArmInitPose = posIntakeArm;
//                        }),
//                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.INIT_POSE)
//                )
//        );
//
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            posIntakeArm -= changeBy;
//                            IntakeArm.intakeArmInitPose = posIntakeArm;
//                        }),
//                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.INIT_POSE)
//                )
//        );

//        //claw
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posIntakeArm += 0.05;
                            IntakEndUnit.IntakeClawOpenPos = posIntakeArm;
                        }),
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                )
        );


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posIntakeArm -= 0.05;
                            IntakEndUnit.IntakeClawOpenPos = posIntakeArm;
                        }),
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                )
        );
//
//
//        ///intake rot
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            posIntakeArm += 0.1;
//                            IntakeEndUnitRotator.initPose = posIntakeArm;
//                        }),
//                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INIT_POSE)
//                )
//        );
//
//
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            posIntakeArm -= 0.1;
//                            IntakeEndUnitRotator.initPose = posIntakeArm;
//                        }),
//                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INIT_POSE)
//                )
//        );
//
        ///scoring intake
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            posIntakeArm += 0.05;
//                            ScoringClawEndUnit.scoringClawClosePos = posIntakeArm;
//                        }),
//                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
//                )
//        );
//
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            posIntakeArm -= 0.05;
//                            ScoringClawEndUnit.scoringClawClosePos = posIntakeArm;
//                        }),
//                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
//                )
//        );

//
//
//        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whenActive(
////                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
//                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw()
//        );
//
//        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05).whenActive(
////                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw()
//                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
//        );
//
////        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
////                new SequentialCommandGroup(
////                       mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET)
////                )
////        );
////
////        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
////                new SequentialCommandGroup(
////                        mmSystems.elevator.moveToPose(Elevator.ElevatorState.ELEVATOR_DOWN)
////                )
////        );
////
////
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posElbow += changeBy;
                            ScoringEndUnitElbow.ElbowScoreSpecimenPose = posElbow;
                        }),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SPECIMEN_POSE)
                )
        );



        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posElbow -= changeBy;
                            ScoringEndUnitElbow.ElbowScoreSpecimenPose = posElbow;
                        }),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SPECIMEN_POSE)
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posArm += changeBy;
                            ScoringArm.scoringArmScorePose = posArm;
                        }),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_ARM_SCORE_POSE)
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posArm -= changeBy;
                            ScoringArm.scoringArmScorePose = posArm;
                        }),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_ARM_SCORE_POSE)
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
        );

                new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whileActiveContinuous(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                );

        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whileActiveContinuous(
                        MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw()
                );

//
//
//
//
//        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                ClimbingCommand.PrepareClimbToThird()
//        );
//
//        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                ClimbingCommand.ClimbToThird()
//        );
//
//        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new InstantCommand(() -> MMRobot.getInstance().mmSystems.elevator.disablePID())
//        );
//
        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whileActiveContinuous(
                        new RunCommand(
                                () -> MMRobot.getInstance().mmSystems.elevator.setPower(mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)),
                                MMRobot.getInstance().mmSystems.elevator
                        )
                ).whenInactive(
                        new RunCommand(
                                () -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0),
                                MMRobot.getInstance().mmSystems.elevator
                        )
                );

        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whileActiveContinuous(
                        new RunCommand(
                                () -> MMRobot.getInstance().mmSystems.elevator.setPower(-mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)),
                                MMRobot.getInstance().mmSystems.elevator
                        )
                ).whenInactive(
                        new RunCommand(
                                () -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0),
                                MMRobot.getInstance().mmSystems.elevator
                        )
                );

        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_ARM_SCORE_SAMPLE)
        );

        /*
        DPAD_LEFT + elbow
        DPAD_RIGHT - elbow
        Y + arm
        A - arm
        X: action
         */



//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                new SequentialCommandGroup(
//                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_ARM_SCORE_POSE),
//                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_SCORE),
//                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET)
//                )
//        );
//
        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor()
        );
        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET)
        );

        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_TRANSFER_POSE)
                )
        );

//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INIT_POSE)
//        );
//                MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                        new ScoringArm_SpeedControll(time,0.19)
//                );
    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();

        MMRobot.getInstance().mmSystems.telemetry.addData("elbow", posElbow);
        MMRobot.getInstance().mmSystems.telemetry.addData("ScoringArm", posArm);
        MMRobot.getInstance().mmSystems.telemetry.addData("intakeArm", posIntakeArm);
//        MMRobot.getInstance().mmSystems.telemetry.addData("ScoringArm2", MMRobot.getInstance().mmSystems.scoringArm.getPosition());

        FtcDashboard.getInstance().getTelemetry().addData("touch sensor back", touchSensors.getStateScoring());
        FtcDashboard.getInstance().getTelemetry().addData("touch sensor front", touchSensors.getStateIntake());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();

        telemetry.update();

    }
}



//if linear and intake arm are opened wait some time before prepare specimen intake