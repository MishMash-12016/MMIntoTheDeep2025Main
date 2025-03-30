package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.HashMap;

@TeleOp
public class TuningTest extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    boolean Specimenintake = true;

    private static double posElbow = ScoringEndUnitElbow.ScoringElbowState.INIT_POSE.position.get();
    private static double posArm = ScoringArm.ScoringArmState.INIT_POSE.position.get();
    private static double posIntakeArm = IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE.position.get();

    HashMap<String, Double> poses = new HashMap<>();


    private static final double changeBy = 0.02;
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

        robotInstance.mmSystems.initRobotSystemsTeleOp();
        robotInstance.mmSystems.initDriveTrain();


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posIntakeArm += changeBy;
                            IntakEndUnit.IntakeClawOpenPos = posIntakeArm;
                        }),
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posIntakeArm -= changeBy;
                            IntakEndUnit.IntakeClawOpenPos = posIntakeArm;
                        }),
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                )
        );

        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whenActive(
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
        );

        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05).whenActive(
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw()
        );


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posElbow += changeBy;
                            ScoringEndUnitElbow.ElbowScoreSpecimenPose = posElbow;
                        }),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SPECIMEN_POSE)
                )
        );


        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                       mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET)
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        mmSystems.elevator.moveToPose(Elevator.ElevatorState.ELEVATOR_DOWN)
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
                            ScoringArm.scoringArmSpecimenSideScore = posArm;
                        }),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_ARM_SCORE_POSE)
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            posArm -= changeBy;
                            ScoringArm.scoringArmSpecimenSideScore = posArm;
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


//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new InstantCommand(() -> changeByButton(MMRobot.getInstance().mmSystems.scoringArm, changeBy, MMRobot.getInstance().mmSystems.scoringArm::setPosition))
//        );

        /*
        DPAD_LEFT + elbow
        DPAD_RIGHT - elbow
        Y + arm
        A - arm
        X: action
         */
    }
    private <T extends SubsystemBase> void changeByButton(T subsystem, double change, java.util.function.Consumer<Double> setPositionMethod) {
        poses.merge(subsystem.getClass().getSimpleName(), change, Double::sum);
        setPositionMethod.accept(poses.get(subsystem.getClass().getSimpleName()));
    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();

        telemetry.addData("elbow", posElbow);
        telemetry.addData("arm", posArm);

        telemetry.addData("intake arm", posIntakeArm);

        telemetry.update();

    }
}