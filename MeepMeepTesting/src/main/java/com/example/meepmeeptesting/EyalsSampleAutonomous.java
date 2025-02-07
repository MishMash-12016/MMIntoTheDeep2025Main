package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class EyalsSampleAutonomous {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(12,14.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(720), Math.toRadians(720), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-39, -65.5, Math.toRadians(180)))


                        //score pre-load
                        .lineToLinearHeading(new Pose2d(-50, -65.5,Math.toRadians(180)))

                        //collect first
                        .lineToSplineHeading(new Pose2d(-48, -45, Math.toRadians(270)))


                        //score first
                        .lineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)))


                        //collect second
                        .lineToLinearHeading(new Pose2d(-58, -45, Math.toRadians(270)))


                        //score second
                        .lineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)))


                        //collect third
                        .lineToLinearHeading(new Pose2d(-50.8, -45, Math.toRadians(315)))


                        //score third
                        .lineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)))


                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
