package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Config
@TeleOp
public class configTeleOp extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    public static double scoringArmPose;
    public static double scoringEndUnitPose;
    public static double scoringElbowPose;
    public static double scoringEndUnitRotatorPose;

    public static double intakeEndUnitPose;
    public static double intakeEndUnitRotatorPose;
    public static double linearIntakePose;
    public static double intakeArmPose;
    public static double elevatorPose;

    public configTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
        scoringArmPose = ScoringArm.scoringArmInitPose;
        scoringElbowPose = ScoringEndUnitElbow.prepareSampleScorePose;
        scoringEndUnitPose = ScoringClawEndUnit.scoringClawOpenPos;
        scoringEndUnitRotatorPose = ScoringEndUnitRotator.rotatorSampleTransferPose;
        intakeEndUnitPose = IntakEndUnit.IntakeClawOpenPos;
        linearIntakePose = 0;
        elevatorPose = Elevator.elevatorDown;
        intakeArmPose = IntakeArm.intakeArmInitPose;
        intakeEndUnitRotatorPose = IntakeEndUnitRotator.initPose;
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;


        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();

        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(scoringElbowPose).schedule();
        MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(scoringEndUnitPose).schedule();
        MMRobot.getInstance().mmSystems.scoringArm.setPosition(scoringArmPose).schedule();
        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(scoringEndUnitRotatorPose).schedule();
        MMRobot.getInstance().mmSystems.intakeArm.setPosition(intakeArmPose).schedule();
        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(intakeEndUnitRotatorPose).schedule();
        MMRobot.getInstance().mmSystems.intakEndUnit.setPose(intakeEndUnitPose).schedule();
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(linearIntakePose).schedule();
        MMRobot.getInstance().mmSystems.elevator.moveToPose(elevatorPose).schedule();


        FtcDashboard.getInstance().getTelemetry().update();
        telemetry.update();
    }
}