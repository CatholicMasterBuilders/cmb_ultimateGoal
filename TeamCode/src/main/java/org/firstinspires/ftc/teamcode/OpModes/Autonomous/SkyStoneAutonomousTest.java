//package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.exception.RobotCoreException;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Func;
//import org.firstinspires.ftc.teamcode.DataTypes.Coordinate;
//import org.firstinspires.ftc.teamcode.DataTypes.Waypoint;
//import org.firstinspires.ftc.teamcode.Robot.Robot;
//
//@Autonomous(name="SkyStoneAutonomousCleanUp", group= "Linear Opmode")
//public class SkyStoneAutonomousTest extends Waypoint_Nav_Base {
//    Robot robot;
//
//    // test configurables
//
//    // Variables
//    int alliance_mirror_adjustor; // Specifies how to adjust field coords and rotations for red and blue
//    int wait_time = 0;
//    int wait_time1 = 0;
//    int wait_time2 = 0;
//
//    public StringBuilder chosenWaypoints = new StringBuilder();
// //   public AutonomousStartLocation autoStartingLocation = AutonomousStartLocation.UNKNOWN;
//
//    @Override
//    public void runOpMode() {
//        robot = new Robot(this);
//
//        /**********************************
//         * Configure the starting location
//         *********************************/
//        telemetry.addLine()
//                .addData("Please select the start location using:", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return String.format( "%s", autoStartingLocation.toString());
//                    }
//                });
//        telemetry.addLine("gamepad2.dpad_up    => RED  Loading");
//        telemetry.addLine("gamepad2.dpad_right => RED  Cleanup");
//        telemetry.addLine("gamepad2.dpad_down  => BLUE Loading");
//        telemetry.addLine("gamepad2.dpad_left  => BLUE Cleanup");
//        Gamepad prev_gamepad1 = new Gamepad();
//        Gamepad prev_gamepad2 = new Gamepad();
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        try {
//            prev_gamepad2.copy(gamepad2);
//            while ( !gamepad2.x && !isStopRequested()) {
//
//                // Specify the starting location of the robot
//                if (CheckForButtonEvent(prev_gamepad2.dpad_up,gamepad2.dpad_up)) {
//                    autoStartingLocation = AutonomousStartLocation.RED_LEFT;
//                }
//
//                if (CheckForButtonEvent(prev_gamepad2.dpad_down,gamepad2.dpad_down)) {
//                    autoStartingLocation = AutonomousStartLocation.BLUE_LEFT;
//                }
//                prev_gamepad2.copy(gamepad2);
//                telemetry.update();
//
//            }
//            while (gamepad2.x && !isStopRequested()) ; // Wait for x button release
//            telemetry.log().clear(); telemetry.clear();
//            telemetry.clearAll();
//
//            // Check to make sure value start location was chosen
////            if (autoStartingLocation == AutonomousStartLocation.UNKNOWN) {
////                telemetry.log().add("Invalid start location.  Try again.");
////                sleep(5000);
////                return;
////            }
//            // Check for early exit of opMode
//            if (isStopRequested()) {
//                return;
//            }
//
//            initializeKnownLocations(robot, autoStartingLocation);
//
//            /**********************************
//             * Configure the waypoints
//             *********************************/
//            chosenWaypoints.append(" ");
//            telemetry.addLine("Choose waypoints ( press gp2.X when done )");
//            telemetry.addLine()
//                    .addData("Chosen waypoints:", new Func<String>() {
//                        @Override
//                        public String value() {
//                            return String.format( "%s", chosenWaypoints.toString());
//                        }
//                    });
//            telemetry.addLine("gamepad1.x => Inner Alliance Bridge");
//            telemetry.addLine("gamepad1.y+Bumper => Outer Alliance Bridge");
//            telemetry.addLine("gamepad1.a => Retrieve Skystone");
//            telemetry.addLine("gamepad1.b => Deliver Skystone");
//            telemetry.addLine("gamepad1.dpad_up => Foundation");
//            telemetry.addLine("gamepad1.dpad_right => Wait( 1sec )");
//
//
//
//            prev_gamepad1.copy(gamepad1);
//            prev_gamepad2.copy(gamepad2);
//            while ( !gamepad2.x && !isStopRequested()) {
//
//                // Add a stop on the inner side of the alliance bridge
//                if (CheckForButtonEvent(prev_gamepad1.x,gamepad1.x)) {
//                    AddInnerBridgeWaypoint();
//                }
//
//                // Add a stop on the outer side of the alliance bridge
//                if (CheckForButtonEvent(prev_gamepad1.y,gamepad1.y)) {
////                    AddOuterBridgeWaypoint(Coordinate.Direction.FORWARD);
//                    AddOuterBridgeWaypoint();
//                }
//
//                // Add a stop to deliver a stone
//                if (CheckForButtonEvent(prev_gamepad1.b,gamepad1.b)) {
//                    AddDeliverSkystoneWaypoint(true );
//                }
//
//                // Add a stop to retrieve the first skystone
//                if (CheckForButtonEvent(prev_gamepad1.a,gamepad1.a)) {
//                    AddRetrieveSkystoneWaypoint( SkystoneIdentifier.S1 );
//                }
//
//                // Add a stop to retrieve the second skystone
//                if (CheckForButtonEvent(prev_gamepad1.dpad_down,gamepad1.dpad_down)) {
//                    AddRetrieveSkystoneWaypoint( SkystoneIdentifier.S2 );
//                }
//
//                //foundation
//                if (CheckForButtonEvent(prev_gamepad1.dpad_up,gamepad1.dpad_up)) {
//                    AddFoundationWaypoint(false);
//                }
//
//                // wait
//                if (CheckForButtonEvent(prev_gamepad1.dpad_right,gamepad1.dpad_right)) {
//                    AddWaitWaypoint();
//                }
//
//                // preset1
//                if (CheckForButtonEvent(prev_gamepad2.a,gamepad2.a)) {
//                    int sleep_time = getWaitTime("Add wait(s) before start: ");
//                    chosenWaypoints.append(sleep_time +"W ");
//
//                    while ( sleep_time-- > 0 ) { AddWaitWaypoint(false);}
//
//                    AddRetrieveSkystoneWaypoint( SkystoneIdentifier.S1, Coordinate.Direction.BACKWARD);
////                    AddBackwardWaypoint();
////                    AddBackwardWaypoint();
//                    AddOuterBridgeWaypoint(Coordinate.Direction.BACKWARD);
//                    AddFoundationDropoffWaypoint(true);
//                    AddOuterBridgeWaypoint(Coordinate.Direction.FORWARD);
//                    telemetry.addLine("Press Gamepad2.x when done");
//                }
//
//                // preset2
//                if (CheckForButtonEvent(prev_gamepad2.b,gamepad2.b)) {
//                    int sleep_time = getWaitTime("Add wait(s) before start: ");
//                    chosenWaypoints.append(sleep_time +"W ");
//                    while ( sleep_time-- > 0 ) { AddWaitWaypoint(false);}
//
//                    AddRetrieveSkystoneWaypoint( SkystoneIdentifier.S1, Coordinate.Direction.BACKWARD);
////                    AddBackwardWaypoint();
////                    AddBackwardWaypoint();
//                    AddOuterBridgeWaypoint(Coordinate.Direction.BACKWARD);
//                    AddFoundationWaypoint(true);
//                    AddOuterBridgeWaypoint(Coordinate.Direction.FORWARD);
//                }
//
//                // preset3
//                if (CheckForButtonEvent(prev_gamepad2.y,gamepad2.y)) {
//                    int sleep_time = getWaitTime("Add wait(s) before start: ");
//                    chosenWaypoints.append(sleep_time +"W ");
//                    while ( sleep_time-- > 0 ) { AddWaitWaypoint(false);}
//
//                    AddRetrieveSkystoneWaypoint(SkystoneIdentifier.S2, Coordinate.Direction.FORWARD);
//                    AddOuterBridgeWaypoint(Coordinate.Direction.BACKWARD);
//                    AddFoundationDropoffWaypoint(true);
//                    AddOuterBridgeWaypoint(Coordinate.Direction.FORWARD);
//                }
//
//                // Add an option to configure bot to go one foot forward
//                if (CheckForButtonEvent(prev_gamepad2.dpad_up,gamepad2.dpad_up)) {
//                    AddForwardWaypoint();
//                }
//                // Add an option to configure bot to go one foot backward
//                if (CheckForButtonEvent(prev_gamepad2.dpad_down,gamepad2.dpad_down)) {
//                    AddBackwardWaypoint();
//                }
//                // Add an option to configure bot to turn 90 degrees left
//                if (CheckForButtonEvent(prev_gamepad2.dpad_left,gamepad2.dpad_left)) {
//                    AddLeftWaypoint();
//                }
//                // Add an option to configure bot to turn 90 degrees right
//                if (CheckForButtonEvent(prev_gamepad2.dpad_right,gamepad2.dpad_right)) {
//                    AddRightWaypoint();
//                }
//
//                prev_gamepad1.copy(gamepad1);
//                prev_gamepad2.copy(gamepad2);
//                telemetry.update();
//            }
//            while (gamepad2.x && !isStopRequested()) ;
//            telemetry.clearAll(); telemetry.update();
//        } catch (RobotCoreException e) {
//            e.printStackTrace();
//        }
//
//        // Check for early exit of opMode
//        if ( isStopRequested() ) { return; }
//
//        // Use the base class to navigate to each of the waypoints
//        super.runOpMode();
//    }
//
//    private static boolean CheckForButtonEvent(boolean prev_state, boolean curr_state) {
//        return !prev_state && curr_state;
//    }
//
//    private int getWaitTime( String caption ) {
//        wait_time = 0;
//        telemetry.addLine()
//                .addData(caption, new Func<String>() {
//                    @Override
//                    public String value() {
//                        return String.format( "%d", wait_time);
//                    }
//                });
//        while ( !isStopRequested() && !gamepad2.right_bumper ) {
//            if ( gamepad2.left_trigger > 0 ) { wait_time++; }
//            if ( gamepad2.right_trigger > 0 ) { wait_time--; }
//            sleep(250);
//            telemetry.update();
//        }
//        while ( !isStopRequested() && gamepad2.right_bumper ) {}//wait till done
//        return wait_time;
//    }
//    private int getWaitTime1( String caption ) {
//        wait_time1 = 0;
//        telemetry.addLine()
//                .addData(caption, new Func<String>() {
//                    @Override
//                    public String value() {
//                        return String.format( "%d", wait_time1);
//                    }
//                });
//        while ( !isStopRequested() && !gamepad2.right_bumper ) {
//            if ( gamepad2.left_trigger > 0 ) { wait_time1++; }
//            if ( gamepad2.right_trigger > 0 ) { wait_time1--; }
//            sleep(250);
//            telemetry.update();
//        }
//        while ( !isStopRequested() && gamepad2.right_bumper ) {}//wait till done
//        return wait_time1;
//    }
//    private int getWaitTime2( String caption ) {
//        wait_time1 = 0;
//        telemetry.addLine()
//                .addData(caption, new Func<String>() {
//                    @Override
//                    public String value() {
//                        return String.format( "%d", wait_time2);
//                    }
//                });
//        while ( !isStopRequested() && !gamepad2.right_bumper ) {
//            if ( gamepad2.left_trigger > 0 ) { wait_time2++; }
//            if ( gamepad2.right_trigger > 0 ) { wait_time2--; }
//            sleep(250);
//            telemetry.update();
//        }
//        return wait_time2;
//    }
//
//    //////////////////////////////////////
//    // Waypoint Coordinates and Runnables
//    //////////////////////////////////////
//    private void AddWaitWaypoint( ) {
//        AddWaitWaypoint(true);
//    }
//    private void AddWaitWaypoint(boolean showcrumb) {
//        // Wait for 1 second but do it in two intervals so it can be interrupted
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                sleep(500);
//            }
//        }));
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                sleep(500);
//            }
//        }));
//        if ( showcrumb ) chosenWaypoints.append("W ");
//    }
//    private void AddMotionWaypoint( final double inches, final double power ) {
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                robot.drivetrain.encoder_drive_inches(inches, inches, power, power);
//            }
//        }));
//    }
//    private void AddTurnWaypoint(final double turn_degrees, final double power ) {
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                robot.drivetrain.turn(turn_degrees,power);
//            }
//        }));
//    }
//
//    private void AddForwardWaypoint() {
//        AddMotionWaypoint(12, 0.25 );
//        chosenWaypoints.append("^ ");
//    }
//    private void AddBackwardWaypoint() {
//        AddMotionWaypoint(-12, 0.25 );
//        chosenWaypoints.append("v ");
//    }
//    private void AddLeftWaypoint() {
//        AddTurnWaypoint(90, 0.3 );
//        chosenWaypoints.append("< ");
//    }
//    private void AddRightWaypoint() {
//        AddTurnWaypoint(-90, 0.3 );
//        chosenWaypoints.append("> ");
//    }
//    private void AddFoundationDropoffWaypoint(final boolean stone_delivery) {
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                // Use position integration because this waypoint is long
//                robot.intake.setFlickerDown();
//                robot.placingSystem.grab();
//                robot.drivetrain.enablePositionIntegrator();
//            }
//        }));
//        waypoints.add(new Waypoint(new Coordinate(1400, (1500) * alliance_mirror_adjustor, Coordinate.Direction.BACKWARD), new Runnable() {
//            @Override
//            public void run() {
//                if (stone_delivery == true) {
//                    robot.intake.setFlickerUp();
//                    robot.sliderArm.toggleSliderLevels();
//                    robot.sliderArm.toggleSliderLevels();
//                    sleep(50);
//                    robot.placingSystem.guideRailFulExtend();
//                    sleep(2000);
//                    robot.placingSystem.release(); // let go og stone
//                    sleep(50);
//                    robot.placingSystem.guideRailIn();
//                    sleep(200);
//                    robot.sliderArm.toggleSliderLevels(true);
//                }
//                robot.maneuver.setDrive_power( 0.4 );
//                robot.drivetrain.disablePositionIntegrator();
//
//            }
//        }));
//        chosenWaypoints.append("FD ");
//        if ( stone_delivery == true ) {chosenWaypoints.append("D ");}
//    }
//    private void AddFoundationWaypoint(final boolean stone_delivery) {
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                // Use position integration because this waypoint is long
//                robot.intake.setFlickerDown();
//                robot.placingSystem.grab();
//                robot.drivetrain.enablePositionIntegrator();
//            }
//        }));
//        // Waypoint in front of foundation to get the robot aligned with its back facing the foundation
//        Coordinate.Direction dir = ( stone_delivery == true ) ? Coordinate.Direction.BACKWARD : Coordinate.Direction.BACKWARD;
//        int x_coord = (stone_delivery) ? 1200 : 1100;
//        int y_coord = (stone_delivery) ? 1030 : 930;
//        waypoints.add(new Waypoint(new Coordinate(x_coord, (y_coord + robot.cog_offset_from_back) * alliance_mirror_adjustor, dir), new Runnable() {
//            @Override
//            public void run() {
//                if (stone_delivery == true) {
//                    robot.intake.setFlickerUp();
//                    robot.sliderArm.toggleSliderLevels();
//                    robot.sliderArm.toggleSliderLevels();
//                    sleep(200);
//                    robot.placingSystem.guideRailFulExtend();
//                }
//                robot.IMU.turn_to_heading(0.25, 90 * alliance_mirror_adjustor);
////                robot.drivetrain.turn_to_heading(0.25, 90 * alliance_mirror_adjustor);
//                robot.maneuver.setDrive_power( 0.25 );
//
//            }
//        }));
//
//        // Waypoint that puts the the back of the robot on the foundation
//        // This will keep the current x coordinate so that we generate no turn
//        y_coord = (stone_delivery) ? ((alliance_mirror_adjustor==-1) ? 750 : 550) : 650;
//        waypoints.add(new Waypoint(new Coordinate(KEEP_CURRENT_COORD, (y_coord /*670*/ + robot.cog_offset_from_back) * alliance_mirror_adjustor, Coordinate.Direction.BACKWARD), new Runnable() {
//            @Override
//            public void run() {
//                if (stone_delivery == true) {
////                    robot.sliderArm.toggleSliderLevels(true);
//                }
//                robot.drivetrain.encoder_drive_inches(-5, -5, 0.07, 0.07);
//                robot.foundationlocks.lock();
//                if (stone_delivery == true) {
//                    robot.placingSystem.release(); // let go og stone
//                }
//
//                sleep(500); // let the locks lock since we aren't waiting for stones
//
//                robot.maneuver.setDrive_power( 0.4 );
//
//                if (stone_delivery == true) {
//                    robot.placingSystem.guideRailIn();
//                }
//                robot.drivetrain.disablePositionIntegrator();
//            }
//        }));
//
//        // Waypoint to put the foundation in place.  Most of the positioning is done in the
//        // Runnable but the offset to x helps clear the wall while turning
//        int x_offset = (alliance_mirror_adjustor==-1) ? 300 : 200;
//        y_coord = (alliance_mirror_adjustor==-1) ? 1500 : 1400;
//        waypoints.add(new Waypoint(new Coordinate(x_coord-300, y_coord /*1200*/ * alliance_mirror_adjustor, Coordinate.Direction.FORWARD), new Runnable() {
//            @Override
//            public void run() {
//                if (stone_delivery == true) {
//                    robot.sliderArm.toggleSliderLevels(true);
//                }
//
//                // The line of code below was contributed by one of the builders
//                // dont crash into the wall <----- unfortunately it wouldn't compile
//
////                robot.IMU.turn_to_heading(0.25, 180);
//                robot.drivetrain.turn_to_heading(0.25, 180);
//
//                robot.foundationlocks.release();
//                sleep(250); // let the locks unlock
//            }
//        }));
//        chosenWaypoints.append("F ");
//        if ( stone_delivery == true ) {chosenWaypoints.append("D ");}
//    }
//
//    private enum SkystoneIdentifier { S1, S2 };
//    private void AddRetrieveSkystoneWaypoint(SkystoneIdentifier id) {
//        AddRetrieveSkystoneWaypoint( id, Coordinate.Direction.FORWARD );
//    }
//    private void AddRetrieveSkystoneWaypoint(SkystoneIdentifier id, final Coordinate.Direction direction ) {
//        // This is just a placeholder for the skystone and will be filled in once the location is known
//        int skystone_x = 0;
//        Coordinate skystoneCoord;
//        if ( id == SkystoneIdentifier.S1 ) {
//            skystone_x = 2;
////            skystoneCoord = SkyStone1Coordinate.clone();
//        }
//        else {
//            skystone_x = 1;
//            // If this is the second skystone, we also need to get back somewhere close to the skystones
//
//            /************************************************************
//             * Why is this so ugly?
//             * This waypoint is supposed to put the robot in a position to get
//             * the second skystone.  We want to come at the skystone from the end, but if the
//             * stone is against the wall, then the angle from the end isn't enough
//             *    Wall  |
//             *          v
//             *    ---------------------------|
//             *                               |
//             *              -- -- -- -- -- --| <--wall
//             *                     ^  ^      |
//             *                 -900  -1000
//             *
//             * The alliance_mirror_adjustor block is because the motion is assymetric so if
//             * we are on the red side, then the above mentioned positioning for S2 needs to be
//             * tweaked to account for the positive y drift that happens because of rotation
//             */
//            int x_coord = -1000;
//            if (skystoneIndex == 0) {
//               x_coord = -900;
//            }
//            int y_offset = 0;
//            if (alliance_mirror_adjustor == -1) {
//                x_coord -= 50; // As this increases, the x coord moves towards the wall
//                 y_offset = 100; // As this increases, the y coord moves towards the wall
//            }
//            waypoints.add(new Waypoint(new Coordinate(x_coord, (1000 + y_offset) * alliance_mirror_adjustor, Coordinate.Direction.BACKWARD), new Runnable() {
//                @Override
//                public void run() {
////                    robot.IMU.turn_to_heading(0.25, -150 * alliance_mirror_adjustor);
//                    robot.drivetrain.turn_to_heading(0.25, -150 * alliance_mirror_adjustor);
//                }
//            }));
//
//        }
//
//        // Runnable to slurp and start with low slow power
//        final SkystoneIdentifier rid = id;
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                robot.intake.setIntakePowers(-0.4);
////                robot.intake.slurp();
////                robot.maneuver.setDrive_power( 0.25 );
//                // Consider using position integration for S2
//                if ( rid == SkystoneIdentifier.S2 ) {
//                    robot.drivetrain.enablePositionIntegrator();
//                    robot.maneuver.setDrive_power( 0.25 );
//                }
//            }
//        }));
//
//
//        int foo =0;
//        foo = getWaitTime1("Add wait(s) before backup from stone: ");
//
//        // Placeholder for the skystone waypoint and the runnable to grab it
//        waypoints.add( new Waypoint( new Coordinate(skystone_x, 0), new Runnable() {
//            @Override
//            public void run() {
//                if (rid == SkystoneIdentifier.S1) {
//                    robot.drivetrain.encoder_drive_inches(18, 0.15);
//                } else {
//                    robot.drivetrain.encoder_drive_inches(8, 0.15);
//                }
////                if (rid == SkystoneIdentifier.S1) {
////                    robot.drivetrain.encoder_drive_inches(3, 0.1);
////                    robot.drivetrain.encoder_drive_inches(4, 0.1);
////                }
//        }}));
//                while ( foo-- > 0 ) {
//                    waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//                        @Override
//                        public void run() {
//                            sleep(500);
//                        }
//                    }));
//                    waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//                        @Override
//                        public void run() {
//                            sleep(500);
//                        }
//                    }));
//                }
//
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                robot.drivetrain.encoder_drive_inches(-24,0.4);
//
//                // This shouldn't be necessary because we should rely on the next waypoint,
//                // but life is full of things that "should" work.
//                if (direction == Coordinate.Direction.FORWARD){
//                    robot.IMU.turn_to_heading(0.25, 0 * alliance_mirror_adjustor);
//                }
//                else {
//                    robot.IMU.turn_to_heading(0.25, 180 * alliance_mirror_adjustor);
//                }
//
//                // Reset the autonomous drive power
////                robot.maneuver.setDrive_power( 0.4 );
//                // reset position integration for S2
//                if ( rid == SkystoneIdentifier.S2 ) {
//                    robot.drivetrain.disablePositionIntegrator();
//                    robot.maneuver.setDrive_power( 0.4 );
//                }
//            }
//        }));
//        chosenWaypoints.append( id );
//        chosenWaypoints.append( " " );
//        chosenWaypoints.append( wait_time1 + "W " );
//
//    }
//
//    private void AddDeliverSkystoneWaypoint(final boolean kick_out_stone) {
//        // The actual delivery waypoint for the skystone
//        waypoints.add(new Waypoint(new Coordinate(400, 1070 /*1000*/ * alliance_mirror_adjustor, Coordinate.Direction.FORWARD), new Runnable() {
//            @Override
//            public void run() {
//                if ( kick_out_stone == true ) {
//                    robot.intake.spit(); // Spit stone out the front
//                    // Wait for it to come out the front
//                    sleep(1000);
//                    robot.intake.salivate();
//                }
//            }
//        }));
//        chosenWaypoints.append("D ");
//    }
//
//    /**
//     * This Delivers the skystone and Navigates to the inner bridge
//     * but assumes that the robot is already at the inner bridge facing the building zone
//     * @param kick_out_stone
//     */
//    private void AddQuickDeliverSkystoneWaypoint(final boolean kick_out_stone) {
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                if ( kick_out_stone == true ) {
//                    robot.intake.spit(); // Spit stone out the front
//                }
//
////                sleep(100); // maybe this will make the drive not stop short
//                robot.drivetrain.debugStall = true;
//                robot.drivetrain.encoder_drive_inches(8, 0.4 );
//
////                if ( kick_out_stone == true ) {
////                    // Wait for it to come out the front
////                    sleep(50);
////                }
//
//                robot.drivetrain.encoder_drive_inches(-12,0.4 );
//                robot.drivetrain.debugStall = false;
//
//                if ( kick_out_stone == true ) {
//                    robot.intake.salivate();
//                }
//            }
//        }));
//        chosenWaypoints.append("D I ");
//    }
//
//    private void AddOuterBridgeWaypoint() {
//        Coordinate.Direction dir = Coordinate.Direction.FORWARD;
////        telemetry.addLine("Direction");
//        while ( !isStopRequested() && !gamepad1.right_bumper && !gamepad1.left_bumper ) {
//
//            sleep(250);
////            telemetry.update();
//        }
//        if ( gamepad1.left_bumper ) { dir = Coordinate.Direction.FORWARD; }
//        if ( gamepad1.right_bumper ) { dir = Coordinate.Direction.BACKWARD; }
//
//        while ( !isStopRequested() && ( gamepad1.right_bumper || gamepad1.left_bumper) ) {}//wait till done
//        AddOuterBridgeWaypoint(dir);
//    }
//    private void AddOuterBridgeWaypoint(Coordinate.Direction dir) {
//        waypoints.add(new Waypoint(new Coordinate(0, /*100*/1500 * alliance_mirror_adjustor, dir)));
//        chosenWaypoints.append("O ");
//    }
//
//    private void AddInnerBridgeWaypoint() {
//        AddInnerBridgeWaypoint(Coordinate.Direction.FORWARD);
//    }
//    private void AddInnerBridgeWaypoint(Coordinate.Direction dir) {
//        // RED drifts towards the middle of the field so add 100 to the waypoint
//        waypoints.add(new Waypoint(new Coordinate(0, ((alliance_mirror_adjustor==1) ? 1000: 1100) * alliance_mirror_adjustor, dir)));
//        chosenWaypoints.append("I ");
//    }
//
//    private void AddSafeParkWaypoint(final boolean dir) {
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                int safe_distance = 0;
//                if ( dir == true ) {
//                    // Use front extension
////                    robot.placingSystem.tape_measure_tongue_out();
//                    safe_distance = 15;
//                } else {
//                    // Use rear extension
////                    robot.placingSystem.guideRailFulExtend();
//                    safe_distance = -20;
//                }
//                robot.drivetrain.encoder_drive_inches(safe_distance,safe_distance,0.4,0.4);
//                robot.intake.salivate();
//            }
//        }));
//
//        chosenWaypoints.append("I ");
//    }
//
//    private void AddIntakeInWaypoint() {
//        // Turn ReturnToThree the intake system
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                robot.intake.slurp();
//            }
//        }));
//    }
//    private void AddIntakeOutWaypoint() {
//        // Turn ReturnToThree the intake system
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                robot.intake.spit();
//            }
//        }));
//    }
//    private void AddIntakeOffWaypoint() {
//        // Turn ReturnToThree the intake system
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                robot.intake.salivate();
//            }
//        }));
//    }
//    private void AddBlockIntakeWaypoint() {
//        // Block the intake system from getting the stone into the robot
//        // so that it can be spit out faster
//        waypoints.add(new Waypoint(NonNavCoordinate, new Runnable() {
//            @Override
//            public void run() {
//                robot.intake.blockInput();
//            }
//        }));
//    }
//}