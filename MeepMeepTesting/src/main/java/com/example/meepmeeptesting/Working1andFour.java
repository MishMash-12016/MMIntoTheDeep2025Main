package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.nio.channels.MembershipKey;

public class Working1andFour {

        //collection from human player
        private static final Pose2d collectionSpecimanPos = new Pose2d(42,-67,Math.toRadians(90));


        //scoring positions
        private static final Vector2d scoreSpecimanPos = new Vector2d(5.5, -27); // when driving to scoring location
        private static final Vector2d scoreSpecimanPos2 = new Vector2d(5.5, -40); // when driving backwards to score
        private static final int scoreSpecimanXConst = 3; // the spacing of the scored specimens


        //pushing to human player
        private static final Pose2d pushSamplePos = new Pose2d(33.24,-38, Math.toRadians(230));
        private static final float pushingSampleXConst = 10; // the spacing between the samples
        /*
         * a adjustive const for each sample might be needed
         */

        //parking position
        private static final Pose2d parkPos = new Pose2d(45, -60, Math.toRadians(90));


        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(700);

            double xPose = -28;
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(11.02,14.5)//14.5
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(100, 100, Math.toRadians(720), Math.toRadians(720), 15)
                    .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(5.5, -62.73, Math.toRadians(90.00)))

                    .setTangent(Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(5.5, -50,Math.toRadians(90)))
                    .splineToLinearHeading(new Pose2d(1, -31,Math.toRadians(90)),Math.toRadians(180))

                    .setTangent(Math.toRadians(-150))
                    .lineToLinearHeading(new Pose2d(1, -32, Math.toRadians(90)))
                    .splineToSplineHeading(new Pose2d(31, -35, Math.toRadians(220)), Math.toRadians(0))
//
                    .setTangent(Math.toRadians(290))
                    .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(160)), Math.toRadians(240))


                    .setTangent(Math.toRadians(300))
                    .splineToLinearHeading(new Pose2d(45.8, -47, Math.toRadians(160)), Math.toRadians(240))

                    .setTangent(Math.toRadians(80))
                    .splineToLinearHeading(new Pose2d(50, -38, Math.toRadians(235)), Math.toRadians(80))

                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(new Pose2d(50, -47, Math.toRadians(90)), Math.toRadians(240))

                    .splineToLinearHeading(new Pose2d(50,-47.5,Math.toRadians(90)), Math.toRadians(270))

                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(50, -59.2,Math.toRadians(90)), Math.toRadians(270))

                    .setTangent(Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(0, -28),Math.toRadians(155))
//                    .splineToConstantHeading(new Vector2d(2, xPose),Math.toRadians(90))


                    .setTangent(Math.toRadians(300))
                    .splineTo(new Vector2d(45, -61),Math.toRadians(-60))

//                    .setTangent(90)
//                    .splineTo(new Vector2d(4, -35),Math.toRadians(-235))
//                    .splineToLinearHeading(new Pose2d(2, xPose,Math.toRadians(90)),Math.toRadians(225))
//
//
//                    .setTangent(Math.toRadians(300))
//                    .splineTo(new Vector2d(45, -61),Math.toRadians(-60))
//
//                    .setTangent(90)
//                    .splineTo(new Vector2d(4, -35),Math.toRadians(-235))
//                    .splineToLinearHeading(new Pose2d(-2, xPose,Math.toRadians(90)),Math.toRadians(200))
////
//                    .setTangent(Math.toRadians(300))
//                    .splineTo(new Vector2d(45, -61),Math.toRadians(-60))
////
//                   .setTangent(90)
//                    .splineTo(new Vector2d(4, -35),Math.toRadians(-235))
//                    .splineToLinearHeading(new Pose2d(-2, xPose,Math.toRadians(90)),Math.toRadians(200))
//
//                    .setTangent(Math.toRadians(300))
//                    .splineTo(new Vector2d(45, -61),Math.toRadians(-60))
//
//                    .setTangent(90)
//                    .splineTo(new Vector2d(4, -35),Math.toRadians(-235))
//                    .splineToLinearHeading(new Pose2d(-2, xPose,Math.toRadians(90)),Math.toRadians(200))
//
//                    .setTangent(Math.toRadians(300))
//                    .splineTo(new Vector2d(45, -61),Math.toRadians(-60))


//
////--
//
//
                            .build());


            meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
