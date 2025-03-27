package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class g extends MMOpMode {
    static MMRobot robotInstance;
    static final double halfOpenClaw = 0.6;
    static final double rotator = 0;
    final double intakeArmPose = 0.59;

    //parking position
    private static final Pose2d dragScoredSpecimenToSide = new Pose2d(-1, -30, Math.toRadians(90)); //side
    private static final double tangentsToScoreSpecimen = 135;
    private static final double tangentsToIntakeSpecimen = 310;
    final Pose2d intakePose = new Pose2d(40, -65.5, Math.toRadians(90));
    static final Pose2d scorePose = new Pose2d(6, -31.8, Math.toRadians(120));
    public static final Vector2d scoreVector = new Vector2d(scorePose.position.x, scorePose.position.y);

    public g() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
//        MMRobot.getInstance().mmSystems.vision.trackRed();
        MMRobot.getInstance().mmSystems.vision.auto();
        MMRobot.getInstance().mmSystems.vision.trackRed();

        Pose2d currentPose = (new Pose2d(0, 0, Math.toRadians(90)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreload = drive.actionBuilder(currentPose)
                .lineToY(-60);
        new SequentialCommandGroup(
                new InstantCommand(),
                new ActionCommand(driveToScorePreload.build()).alongWith(
                        new SequentialCommandGroup(
                             new WaitCommand(400),
                                new InstantCommand(()->drive.slash_tp(new Pose2d(new Vector2d(0, -60), Math.toRadians(90))))
                        )
                )
        ).schedule();

    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("linear", MMRobot.getInstance().mmSystems.linearIntake.getPosition());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
