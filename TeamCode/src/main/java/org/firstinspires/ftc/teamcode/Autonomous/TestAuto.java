package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class TestAuto extends MMOpMode {
    MMRobot robotInstance;
    public TestAuto() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = (new Pose2d(5.5, -65.5, Math.toRadians(90)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        final Vector2d scoreSpecimenPose = new Vector2d(5.5, -27); // when driving to scoring location
        final Vector2d scoreSpecimenPos2 = new Vector2d(5.5, -40); // when driving backwards to score
        final Pose2d pushSamplePos = new Pose2d(32.8,-38, Math.toRadians(230));
        final double pushingSampleXConst= 10;
        final double halfOpenClaw =0.68;
        final double rotator =0.65;

        TrajectoryActionBuilder driveToScorePreloadSpecimen = drive.actionBuilder(currentPose)
                .splineToConstantHeading(scoreSpecimenPose,Math.toRadians(90));
        TrajectoryActionBuilder driveToScorePreloadSpecimen2 = driveToScorePreloadSpecimen.endTrajectory().fresh()
                .splineToConstantHeading(scoreSpecimenPos2,Math.toRadians(270)); //preload
        TrajectoryActionBuilder driveToPush1 = driveToScorePreloadSpecimen2.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(pushSamplePos, Math.toRadians(0));
         TrajectoryActionBuilder turnRobot =driveToPush1.endTrajectory().fresh()
                 .setTangent(Math.toRadians(300))
                 .splineToLinearHeading(new Pose2d(32.8,-53, Math.toRadians(120)),Math.toRadians(240),new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*0.5));
        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(38.8,-38,Math.toRadians(230)),Math.toRadians(70));

    TrajectoryActionBuilder turnRobot2 =driveToPush2.endTrajectory().fresh()
            .setTangent(Math.toRadians(300))
            .splineToLinearHeading(new Pose2d(38.8,-53, Math.toRadians(120)),Math.toRadians(240),new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*0.5));
        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSpecimen.build()).alongWith(
                        ScoringSpecimanCommand.SpecimanScore()),
                new ActionCommand(driveToScorePreloadSpecimen2.build()),
                new ActionCommand(driveToPush1.build()).alongWith(
                    new ParallelCommandGroup(
                    robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                            robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                            robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                            robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw))),
                robotInstance.mmSystems.intakeArm.setPosition(0.62),
                new WaitCommand(500),
                new ActionCommand(turnRobot.build()),
                new ActionCommand(driveToPush2.build()).alongWith(
                        new ParallelCommandGroup(
                                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                                robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw))),
                robotInstance.mmSystems.intakeArm.setPosition(0.62),
                new WaitCommand(800),
                new ActionCommand(turnRobot2.build())

 ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("linear",MMRobot.getInstance().mmSystems.linearIntake.getPosition());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

}

