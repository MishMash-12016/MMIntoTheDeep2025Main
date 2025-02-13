package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

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

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(11.02,14.5)//14.5
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(100, 100, Math.toRadians(720), Math.toRadians(720), 15)
                    .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(5.5, -62.73, Math.toRadians(90.00)))

                            .splineToConstantHeading(new Vector2d(5.5, -30), Math.toRadians(90))
//--
                            .setTangent(Math.toRadians(260))
                            .lineToLinearHeading(new Pose2d(5.5, -33, Math.toRadians(90)))
                            .splineToSplineHeading(new Pose2d(28, -38, Math.toRadians(235)), Math.toRadians(0))

                            .setTangent(Math.toRadians(290))
                            .splineToLinearHeading(new Pose2d(32.8, -47, Math.toRadians(140)), Math.toRadians(240))// -47 ---- 140
////--
                            .setTangent(Math.toRadians(80))
                            .splineToLinearHeading(new Pose2d(38, -38, Math.toRadians(235)), Math.toRadians(70))

                            .setTangent(Math.toRadians(300))
                            .splineToLinearHeading(new Pose2d(42.8, -47, Math.toRadians(140)), Math.toRadians(240))
////--
                            .setTangent(Math.toRadians(80))
                            .splineToLinearHeading(new Pose2d(48, -38, Math.toRadians(235)), Math.toRadians(70))

                            .setTangent(Math.toRadians(280))
                            .splineToLinearHeading(new Pose2d(47, -47, Math.toRadians(90)), Math.toRadians(270))
//--

                            //first
                            .setTangent(Math.toRadians(270))
                            .lineToLinearHeading(new Pose2d(47, -58.2, Math.toRadians(90)))

                            .lineToLinearHeading(new Pose2d(2, -28,Math.toRadians(90)))

                            //second
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(new Pose2d(12,-38,Math.toRadians(90)), Math.atan((-38.0+58.2)/(12.0-45.0)))
                            .lineToLinearHeading(new Pose2d(45,-58.2,Math.toRadians(90)))

                            .lineToLinearHeading(new Pose2d(0, -28, Math.toRadians(90)))

                            //third
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(new Pose2d(10,-38,Math.toRadians(90)), Math.atan((-38.0+58.2)/(10.0-45.0)))
                            .lineToLinearHeading(new Pose2d(45,-58.2,Math.toRadians(90)))

                            .lineToLinearHeading(new Pose2d(-2, -28, Math.toRadians(90)))

                            //forth
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(new Pose2d(8,-38,Math.toRadians(90)), Math.atan((-38.0+58.2)/(8.0-45.0)))
                            .lineToLinearHeading(new Pose2d(45,-58.2,Math.toRadians(90)))

                            .lineToLinearHeading(new Pose2d(-4, -28, Math.toRadians(90)))

                            //park
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(new Pose2d(6,-40,Math.toRadians(90)), Math.atan((-38.0+58)/(6.0-40)))
                            .lineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)))
                            .build());


            meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
