package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class EyalsSampleAutonomous {

    private static final Pose2d scorePose = new Pose2d(-61, -48.5, Math.toRadians(-102.56));
    private static final Pose2d collectionPose = new Pose2d(-24, 0, Math.toRadians(180));
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(11.417,16.535)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(720), Math.toRadians(720), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-39, -65.5, Math.toRadians(180)))

                                .lineToLinearHeading(new Pose2d(-48, -65.5, Math.toRadians(180)))

                        .lineToSplineHeading(new Pose2d(-58.9, -46., Math.toRadians(-112.3)))


                        .lineToSplineHeading(new Pose2d(-62.9, -48.5, Math.toRadians(-102.56)))



                        .lineToLinearHeading(new Pose2d(-62.5, -47.28, Math.toRadians(-75.16)))

                        .setTangent(Math.toRadians(67))
                        .lineToLinearHeading(new Pose2d(-62.9, -48.5, Math.toRadians(-102.56)))



                        .setTangent(Math.toRadians(67))
                        .lineToLinearHeading(collectionPose)
                        .lineToLinearHeading(scorePose)

                        .setTangent(Math.toRadians(67))
                        .lineToLinearHeading(collectionPose)
                        .lineToLinearHeading(scorePose)

                        .setTangent(Math.toRadians(67))
                        .lineToLinearHeading(collectionPose.plus(new Pose2d(0,0,Math.toRadians(180))))


                        .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


