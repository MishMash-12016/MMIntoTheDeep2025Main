package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class EyalsSampleAutonomous {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(11.417,16.535)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(720), Math.toRadians(720), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(5.5, -65, Math.toRadians(270)))


                        .setTangent(90)
                        .lineToLinearHeading(new Pose2d(5.5, -33, Math.toRadians(270)))

/*
     -----------------------
        pushing
     -----------------------
*/
        //Push first specimen
                .setTangent(Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(5.5, -36, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(31, -38, Math.toRadians(235)), Math.toRadians(0))
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(160)), Math.toRadians(240))
        //Push second specimen

//                        .lineToLinearHeading(new Pose2d(5.5, -33, Math.toRadians(270)))
///*
//     -----------------------
//        pushing
//     -----------------------
//*/
//        //Push first specimen
//                .setTangent(Math.toRadians(270))
//                .lineToLinearHeading(new Pose2d(5.5, -36, Math.toRadians(270)))
//                .splineToLinearHeading(new Pose2d(31, -38, Math.toRadians(235)), Math.toRadians(0))
//                .setTangent(Math.toRadians(290))
//                .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(160)), Math.toRadians(240))
//        //Push second specimen
//                .setTangent(Math.toRadians(80))
//                .splineToLinearHeading(new Pose2d(40, -38, Math.toRadians(235)), Math.toRadians(70))
//                .setTangent(Math.toRadians(300))
//                .splineToLinearHeading(new Pose2d(45.8, -47, Math.toRadians(160)), Math.toRadians(240))
//        //Push third specimen
//                .setTangent(Math.toRadians(80))
//                .splineToLinearHeading(new Pose2d(50, -38, Math.toRadians(235)), Math.toRadians(80))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(50, -47, Math.toRadians(90)), Math.toRadians(240))
//
//                .splineToLinearHeading(new Pose2d(50,-47.5,Math.toRadians(90)), Math.toRadians(270))
///*
//     -----------------------
//        intake & scoring
//     -----------------------
//*/
//
//
//        //First specimen
//                .setTangent(Math.toRadians(270))
//                .lineToLinearHeading(new Pose2d(50, -59.2,Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(3, -28, Math.toRadians(90)))
//
//        //Second specimen
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
//                .lineToLinearHeading(new Pose2d(45, -59.2, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(1, -28, Math.toRadians(90)))
//
//        //Third specimen
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
//                .lineToLinearHeading(new Pose2d(45, -59.2, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(-1, -28, Math.toRadians(90)))
//
//        //Forth specimen
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
//                .lineToLinearHeading(new Pose2d(45, -59.2, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(-3, -28, Math.toRadians(90)))
//
//        //Fifth specimen
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
//                .lineToLinearHeading(new Pose2d(45, -59.2, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(-5, -28, Math.toRadians(90)))
//
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(12, -42, Math.toRadians(90)), Math.atan((-42.0 + 57) / (12.0 - 45)))
//                .lineToLinearHeading(new Pose2d(45, -57, Math.toRadians(90)))

                        .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


