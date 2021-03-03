//package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.exception.RobotCoreException;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Func;
//import org.firstinspires.ftc.teamcode.DataTypes.Coordinate;
//import org.firstinspires.ftc.teamcode.DataTypes.Waypoint;
//import org.firstinspires.ftc.teamcode.Robot.Robot;
//
//import static org.firstinspires.ftc.teamcode.Navigation.SkyStoneField.SkyStoneStartLocation.*;
//import static org.firstinspires.ftc.teamcode.Navigation.SkyStoneField.getAlliance;
//
//@Autonomous(name="SkyStoneAutonomousEnc", group= "Linear Opmode")
//@Disabled
//public class SkyStoneAutonomousEnc extends Waypoint_Nav_BaseEnc {
//    Robot robot;
//    SkyStoneField field;
//
//    // test configurables
//
//    // Variables
//    int alliance_mirror_adjustor; // Specifies how to adjust field coords and rotations for red and blue
//
//    public StringBuilder chosenWaypoints = new StringBuilder();
//    public SkyStoneField.SkyStoneStartLocation autoStartingLocation = UNKNOWN;
//
//    @Override
//    public void runOpMode() {
//        robot = new Robot(this);
//
//        field = new SkyStoneField();
//        field.createField( robot );
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
//        telemetry.addLine("gamepad2.dpad_right => RED  Building");
//        telemetry.addLine("gamepad2.dpad_down  => BLUE Loading");
//        telemetry.addLine("gamepad2.dpad_left  => BLUE Building");
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
//                    autoStartingLocation = RED_DEPOT;
//                }
//                if (CheckForButtonEvent(prev_gamepad2.dpad_right,gamepad2.dpad_right)) {
//                    autoStartingLocation = RED_BUILDING;
//                }
//                if (CheckForButtonEvent(prev_gamepad2.dpad_down,gamepad2.dpad_down)) {
//                    autoStartingLocation = BLUE_DEPOT;
//                }
//                if (CheckForButtonEvent(prev_gamepad2.dpad_left,gamepad2.dpad_left)) {
//                    autoStartingLocation = BLUE_BUILDING;
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
//            if (autoStartingLocation == UNKNOWN) {
//                telemetry.log().add("Invalid start location.  Try again.");
//                sleep(5000);
//                return;
//            }
//            // Check for early exit of opMode
//            if (isStopRequested()) {
//                return;
//            }
//
//            field.initializeKnownLocations(robot, autoStartingLocation);
//            super.initialize( robot );
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
//            telemetry.addLine("gamepad1.y => Outer Alliance Bridge");
//            telemetry.addLine("gamepad1.a => Retrieve Skystone");
//            telemetry.addLine("gamepad1.b => Deliver Skystone");
//            telemetry.addLine("gamepad1.dpad_up => Foundation");
//            telemetry.addLine("gamepad1.dpad_right => Wait( 1sec )");
//
//            if ( getAlliance(autoStartingLocation) == SkyStoneField.Alliance.RED ) {
//                alliance_mirror_adjustor = -1; // RED y coords are negative
//            }
//            else {
//                alliance_mirror_adjustor = 1; // BLUE y coords are positive
//            }
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
//                    AddOuterBridgeWaypoint(true);
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
//                    AddRetrieveSkystoneWaypoint( SkystoneIdentifier.S1, Coordinate.Direction.FORWARD);
//                    AddIntakeOffWaypoint();
//                    AddInnerBridgeWaypoint(Coordinate.Direction.FORWARD);
//                    AddQuickDeliverSkystoneWaypoint(false); // deliver and park
//                }
//
//                // preset2
//                if (CheckForButtonEvent(prev_gamepad2.b,gamepad2.b)) {
//                    AddRetrieveSkystoneWaypoint( SkystoneIdentifier.S1, Coordinate.Direction.BACKWARD );
//                    AddInnerBridgeWaypoint(Coordinate.Direction.BACKWARD);
////                    AddDeliverSkystoneWaypoint();
//                    AddFoundationWaypoint(true);
//                    AddOuterBridgeWaypoint(false);
//                }
//
//                // preset3
//                if (CheckForButtonEvent(prev_gamepad2.y,gamepad2.y)) {
//                    AddBlockIntakeWaypoint();
//                    AddRetrieveSkystoneWaypoint( SkystoneIdentifier.S1, Coordinate.Direction.FORWARD );
//                    AddIntakeOffWaypoint();
//
//                    AddInnerBridgeWaypoint( Coordinate.Direction.FORWARD );
//                    AddQuickDeliverSkystoneWaypoint(true );
//                    AddRetrieveSkystoneWaypoint(SkystoneIdentifier.S2, Coordinate.Direction.FORWARD);
//                    AddIntakeOffWaypoint();
//                    AddInnerBridgeWaypoint( Coordinate.Direction.FORWARD );
//                    AddQuickDeliverSkystoneWaypoint(false );
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
//    //////////////////////////////////////
//    // Waypoint Coordinates and Runnables
//    //////////////////////////////////////
//    private void AddWaitWaypoint() {
//        // Wait for 1 second but do it in two intervals so it can be interrupted
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767, Coordinate.Direction.FORWARD), new Runnable() {
//            @Override
//            public void run() {
//                sleep(500);
//            }
//        }));
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767, Coordinate.Direction.FORWARD), new Runnable() {
//            @Override
//            public void run() {
//                sleep(500);
//            }
//        }));
//        chosenWaypoints.append("W ");
//    }
//    private void AddMotionWaypoint( final double inches, final double power ) {
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767 ), new Runnable() {
//            @Override
//            public void run() {
//                robot.drivetrain.encoder_drive_inches(inches, inches, power, power);
//            }
//        }));
//    }
//    private void AddTurnWaypoint(final double turn_degrees, final double power ) {
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767 ), new Runnable() {
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
//
//    private void AddFoundationWaypoint(final boolean stone_delivery) {
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767, Coordinate.Direction.FORWARD), new Runnable() {
//            @Override
//            public void run() {
//                // Use position integration because this waypoint is long
//                robot.drivetrain.enablePositionIntegrator();
//            }
//        }));
//        // Waypoint in front of foundation to get the robot aligned with its back facing the foundation
//        Coordinate.Direction dir = ( stone_delivery == true ) ? Coordinate.Direction.BACKWARD : Coordinate.Direction.BACKWARD;
//        int x_coord = 1150;
//        waypoints.add(new Waypoint(new Coordinate(x_coord, (1030 + robot.cog_offset_from_back) * alliance_mirror_adjustor, dir), new Runnable() {
//            @Override
//            public void run() {
//                robot.IMU.turn_to_heading(0.25, 90 * alliance_mirror_adjustor);
//                if (stone_delivery == true) {
//                    robot.placingSystem.guideRailFulExtend();
//                }
//            }
//        }));
//
//        // Waypoint that puts the the back of the robot on the foundation
//        // This will keep the current x coordinate so that we generate no turn
//        x_coord = KEEP_CURRENT_COORD;
//        waypoints.add(new Waypoint(new Coordinate(x_coord, (900 /*670*/ + robot.cog_offset_from_back) * alliance_mirror_adjustor, Coordinate.Direction.BACKWARD), new Runnable() {
//            @Override
//            public void run() {
//                robot.drivetrain.encoder_drive_inches(-5, -5, 0.07, 0.07);
//                robot.foundationlocks.lock();
//                if (stone_delivery == true) {
//                    robot.sliderArm.toggleSliderLevels(); // First level
//                    robot.sliderArm.toggleSliderLevels(); // Second level to clear stone
//                    sleep(1000); // let it get to the second level
//                    robot.placingSystem.guideRailIn();
//                }
//                else {
//                    sleep(500); // let the locks lock since we aren't waiting for stones
//                }
//                robot.drivetrain.disablePositionIntegrator();
//            }
//        }));
//
//        // Waypoint to put the foundation in place.  Most of the positioning is done in the
//        // Runnable but the offset to x helps clear the wall while turning
//        waypoints.add(new Waypoint(new Coordinate(1150-100, 1300 /*1200*/ * alliance_mirror_adjustor, Coordinate.Direction.FORWARD), new Runnable() {
//            @Override
//            public void run() {
//                if (stone_delivery == true) {
//                    robot.sliderArm.toggleSliderLevels(true);
//                }
////                robot.IMU.turn(0.25, 35 * alliance_mirror_adjustor);
////                robot.drivetrain.encoder_drive(5, 5, 0.2, 0.2, 0);
//
//                // The line of code below was contributed by one of the builders
//                // dont crash into the wall <----- unfortunately it wouldn't compile
//
//                robot.IMU.turn_to_heading(0.25, 180);
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
//        if ( id == SkystoneIdentifier.S1 ) {
//            skystone_x = 0;
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
////            if (skystoneIndex == 0) {
//            if (field.quarry.getSkystoneIndex() == 0) {
//               x_coord = -900;
//            }
//            int y_offset = 0;
//            if (alliance_mirror_adjustor == -1) {
//                x_coord -= 50; // As this increases, the x coord moves towards the wall
//                 y_offset = 200; // As this increases, the y coord moves towards the wall
//            }
//            waypoints.add(new Waypoint(new Coordinate(x_coord, (1000 + y_offset) * alliance_mirror_adjustor, Coordinate.Direction.BACKWARD), new Runnable() {
//                @Override
//                public void run() {
//                    robot.IMU.turn_to_heading(0.25, -150 * alliance_mirror_adjustor);
//                }
//            }));
//
//        }
//
//        // Runnable to slurp and start with low slow power
//        final SkystoneIdentifier rid = id;
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767, Coordinate.Direction.FORWARD), new Runnable() {
//            @Override
//            public void run() {
//                robot.intake.slurp();
//                robot.maneuver.setDrive_power( 0.25 );
//                // Consider using position integration for S2
//                if ( rid == SkystoneIdentifier.S2 ) {
//                    robot.drivetrain.enablePositionIntegrator();
//                }
//            }
//        }));
//
//        // Placeholder for the skystone waypoint and the runnable to grab it
//        waypoints.add( new Waypoint( new Coordinate(skystone_x, 0), new Runnable() {
//            @Override
//            public void run() {
//                robot.drivetrain.encoder_drive_inches(3,0.1);
//                if (rid == SkystoneIdentifier.S1) {
//                    robot.drivetrain.encoder_drive_inches(3, 0.1);
//                    robot.drivetrain.encoder_drive_inches(4, 0.1);
//                }
//
//                robot.drivetrain.encoder_drive_inches(-20,0.3);
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
//                robot.maneuver.setDrive_power( 0.4 );
//                // reset position integration for S2
//                if ( rid == SkystoneIdentifier.S2 ) {
//                    robot.drivetrain.disablePositionIntegrator();
//                }
//            }
//        }));
//        chosenWaypoints.append( id );
//        chosenWaypoints.append( " " );
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
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767), new Runnable() {
//            @Override
//            public void run() {
//                if ( kick_out_stone == true ) {
//                    robot.intake.spit(); // Spit stone out the front
//                }
//
//                sleep(100); // maybe this will make the drive not stop short
//                robot.drivetrain.encoder_drive_inches(8, 0.4 );
//
//                if ( kick_out_stone == true ) {
//                    // Wait for it to come out the front
//                    sleep(50);
//                }
//
//                robot.drivetrain.encoder_drive_inches(-12,0.4 );
//
//                if ( kick_out_stone == true ) {
//                    robot.intake.salivate();
//                }
//            }
//        }));
//        chosenWaypoints.append("D I ");
//    }
//
//    private void AddOuterBridgeWaypoint(boolean dir) {
//        waypoints.add(new Waypoint(new Coordinate(150, /*100*/1600 * alliance_mirror_adjustor, (dir == true)? Coordinate.Direction.FORWARD: Coordinate.Direction.BACKWARD)));
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
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767), new Runnable() {
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
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767), new Runnable() {
//            @Override
//            public void run() {
//                robot.intake.slurp();
//            }
//        }));
//    }
//    private void AddIntakeOutWaypoint() {
//        // Turn ReturnToThree the intake system
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767), new Runnable() {
//            @Override
//            public void run() {
//                robot.intake.spit();
//            }
//        }));
//    }
//    private void AddIntakeOffWaypoint() {
//        // Turn ReturnToThree the intake system
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767), new Runnable() {
//            @Override
//            public void run() {
//                robot.intake.salivate();
//            }
//        }));
//    }
//    private void AddBlockIntakeWaypoint() {
//        // Block the intake system from getting the stone into the robot
//        // so that it can be spit out faster
//        waypoints.add(new Waypoint(new Coordinate(12767, 12767), new Runnable() {
//            @Override
//            public void run() {
//               // robot.placingSystem.blockInput();
//            }
//        }));
//    }
//}