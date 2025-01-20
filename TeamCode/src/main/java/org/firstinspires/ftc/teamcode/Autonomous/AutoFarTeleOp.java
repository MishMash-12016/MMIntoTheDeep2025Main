package org.firstinspires.ftc.teamcode.Autonomous;

//imports

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.Collections;


@Autonomous
public class AutoFarTeleOp extends MMOpMode {
    MMRobot robotInstance;

    public AutoFarTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        Pose2d currentPose = (new Pose2d(5.5, -65.5, Math.toRadians(90.00)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load

        TrajectoryActionBuilder driveToScorePreloadSpecimen = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(5, -30, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder driveToScorePreLoadSpecimen2 = driveToScorePreloadSpecimen.endTrajectory().fresh()
                .lineToY(-50);
        TrajectoryActionBuilder driveToPickUpFirst = driveToScorePreLoadSpecimen2.endTrajectory().fresh()
                .setTangent(Math.toRadians(360))
                .splineToLinearHeading(new Pose2d(48.84, -44.84, Math.toRadians(270)), Math.toRadians(0));
        TrajectoryActionBuilder sampleIntoSpecimen = driveToPickUpFirst.endTrajectory().fresh()
                .turn(Math.toRadians(180));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = sampleIntoSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(5, -30, Math.toRadians(90)), Math.toRadians(90));

        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSpecimen.build()),
                ScoringSpecimanCommand.SpecimanScore(),
                new WaitCommand(500),
                new ActionCommand(driveToScorePreLoadSpecimen2.build())
                        .alongWith(new WaitCommand(350),
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()),
                new ActionCommand(driveToPickUpFirst.build()),
                //IntakeSampleCommand.prepareSampleIntake(()-> LinearIntake.maxOpening),

        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        FtcDashboard.getInstance().getTelemetry().addData("height", MMRobot.getInstance().mmSystems.elevator.getHeight());
        FtcDashboard.getInstance().getTelemetry().addData("target pose:", MMRobot.getInstance().mmSystems.elevator.targetPose);
        FtcDashboard.getInstance().getTelemetry().update();
    }

}

}
