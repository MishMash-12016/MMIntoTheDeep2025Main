package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class MMsplineTest extends MMOpMode {
    public MMsplineTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    MMRobot robotInstance;


    @Override
    public void onInit() {
        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        Pose2d currentPose = (new Pose2d(5.5, -65.5/0.67, Math.toRadians(90)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(currentPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(30, 60), Math.PI/2)
                        .build());
}}
