/*
 * =============================================================
 *   MobBob Control Program - Software Serial Bluetooth Version
 *   by Kevin Chan (aka Cevinius)
 * =============================================================
 *
 * This program enables MobBob to be controlled using serial commands. In this version of the code, the
 * commands are received over a software serial port, with pins defined in the #define near the top.
 * This means you can use any Arduino compatible board, and plug a bluetooth card into the pins set for
 * software serial. (As opposed to the other version of this designed for the Bluno board from DFRobot.)
 *
 * This program is long and contains 2 main components - a smooth servo animation program and a serial
 * command parser program.
 *
 * Animation System
 * ================
 * The animation program is designed to animate servo keyframe arrays smoothly. The code tries to do its
 * best to be easy to use.
 *
 * The animation system will only queue 1 command. i.e. One command can be running,
 * and one command can be queued up. If you send more commands, they will over-write the queued command.
 *
 * The animation system will by default wait to finish the current animation before starting the next. This
 * means that if the animation data ends with the robot in its base pose, things will join smoothly. To
 * support this, the animation system also has a feature where an animation can have a "finish sequence"
 * to put the robot back into the base pose. This feature is used for the walk forward/backward animations.
 * Those animations have a final sequence which puts the robot back into the base pose.
 *
 * When an animation is finished playing, the animation system will output a response string to the Serial port.
 * This enables the callers to know when the animations they've requested have finished playing. This is useful
 * for users to sequence animations - waiting for one to finish before starting another.
 *
 * The animation code has many variables to enable things to be tweaked. E.g. Update frequency, arduino pins, etc.
 *
 * The animation data array format is also designed to be easy to edit by hand.
 *
 * Command Parser
 * ==============
 * This system parses commands received over serial, and processes them. The commands include one for directly
 * setting servo positions, as well as commands for triggering pre-defined animations and walks.
 *
 * So, users who don't want to worry about the details of walking can just use the pre-defined walks/animations.
 * And, users who want complete control over the servos (to create new animations on the fly) can do that too.
 *
 * As mentioned above, these commands can be used interactively from the Arduino Serial Monitor. They can also be
 * sent in using Bluetooth LE (when a Bluno is used). The phone app will send the commands over Bluetooth LE to the
 * Bluno.
 *
 * General Commands:
 * -----------------
 *   Ready/OK Check: <OK>
 *     Status check. The response is returned immediately to check if the controller is working.
 *
 *   Set Servo: <SV, time, leftHip, leftFoot, rightHip, rightFoot>
 *                time      - time to tween to specified angles, 0 will immediately jump to angles
 *                leftHip   - microsecs from centre. -ve is hip in, +ve is hip out 
 *                leftFoot  - microsecs from flat. -ve is foot down, +ve is foot up
 *                rightHip  - microsecs from centre. -ve is hip in, +ve is hip out 
 *                rightFoot - microsecs from flat. -ve is foot down, +ve is foot up
 *     This command is used to get full control over the servos. You can tween the robot from its
 *     current pose to the specified pose over the duration specified.
 *
 *   Stop/Reset: <ST>
 *     Stops the robot after the current animation. Can be used to stop animations set to loop
 *     indefinitely. This can also be used to put the robot into its base pose (standing straight)
 *
 *   Stop Immediate: <SI>
 *     Stops the robot immediately without waiting to complete the current animation. This
 *     interrupts the robots current animation. Potentially the robot can be mid-animation
 *     and in an unstable pose, so be careful when using this.
 *
 * Standard Walk Commands:
 * -----------------------
 *   Forward:    <FW, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *   Backward:   <BW, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *   Turn Left:  <LT, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *   Turn Right: <RT, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *
 * Fun Animation Commands:
 * -----------------------
 *   Shake Head:      <SX, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *
 *   Bounce:          <BX, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *
 *   Wobble:          <WX, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *   Wobble Left:     <WY, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *   Wobble Right:    <WZ, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *
 *   Tap Feet:        <TX, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *   Tap Left Foot:   <TY, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *   Tap Right Foot:  <TZ, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *
 *   Shake Legs:      <LX, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *   Shake Left Leg:  <LY, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *   Shake Right Leg: <LZ, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
 *
 * Also, the code will send a response string back over Serial when commands have finished 
 * executing. 
 *
 * If the command finished normally, the response string is the command code without
 * parameters. E.g. When it has finished moving forward, it will send the response "<FW>".
 *
 * If a command was interrupted with <SI>, the current animation may have been stopped midway.
 * In this case, the robot could be in a weird mid-way pose, and finishAnims may not have been
 * played. To let the user know this has happened, the response string will have the 
 * parameter -1. E.g If a walk was stopped midway using <SI>, the response string would be
 * <FW,-1> to indicate that the walk has stopped, but it was stopped midway.
 * (Note: If you use <ST> to stop, that will wait for the current animation cycle to complete
 * before stopping. So, animations won't get stopped midway in that case.)
 * 
 * Because the responses are sent after an animation is complete, the command sender can
 * look for the response strings to determine when the robot is ready for a new command.
 * E.g. If you use the command <FW,3>, the response string isn't sent until all 3 steps
 * (and finish anim) are completed. So, the command sender can wait for the response
 * string before telling the robot to do the next thing.
 */
 
#include <Servo.h>
#include <SoftwareSerial.h>

//----------------------------------------------------------------------------------
// Speed of serial communication - Set this for your serial (bluetooth) card.
//----------------------------------------------------------------------------------

// Serial communication speed with the bluetooth board.
// Some boards default to 9600. The board I have has a default value of 115200.
#define SERIAL_SPEED 115200

// Setup a Software Serial port on these pins.
const int rxPin = 11; // pin used to receive data
const int txPin = 12; // pin used to send data
SoftwareSerial softwareSerial(rxPin, txPin);


//----------------------------------------------------------------------------------
// Setup Arduino Pins - Set these for your particular robot.
//----------------------------------------------------------------------------------

const int SERVO_LEFT_HIP   = 5;
const int SERVO_LEFT_FOOT  = 2;
const int SERVO_RIGHT_HIP  = 3;
const int SERVO_RIGHT_FOOT = 4;

// I want this code to be usable on all 4-servo bipeds! (Like Bob, MobBob)
// I noticed that some builds mount the hip servos facing a different
// way to how I did MobBob's so, this setting lets you configure the code
// for either build style.
// 1  for MobBob style front facing hips (joint towards the front)
// -1 for Bob style back facing hips (joint towards the back)
#define FRONT_JOINT_HIPS 1


//----------------------------------------------------------------------------------
// Servo Max/Min/Centre Constants - Set these for your particular robot.
//----------------------------------------------------------------------------------

const int LEFT_HIP_CENTRE = 1580;
const int LEFT_HIP_MIN    = LEFT_HIP_CENTRE - 500;
const int LEFT_HIP_MAX    = LEFT_HIP_CENTRE + 500;

const int LEFT_FOOT_CENTRE = 1410;
const int LEFT_FOOT_MIN    = LEFT_FOOT_CENTRE - 500;
const int LEFT_FOOT_MAX    = LEFT_FOOT_CENTRE + 500;

const int RIGHT_HIP_CENTRE = 1500;
const int RIGHT_HIP_MIN    = RIGHT_HIP_CENTRE - 500;
const int RIGHT_HIP_MAX    = RIGHT_HIP_CENTRE + 500;

const int RIGHT_FOOT_CENTRE = 1465;
const int RIGHT_FOOT_MIN    = RIGHT_FOOT_CENTRE - 500;
const int RIGHT_FOOT_MAX    = RIGHT_FOOT_CENTRE + 500;


//----------------------------------------------------------------------------------
// Keyframe animation data for standard walking gait and other servo animations
//
// Format is { <millseconds>, <leftHipMicros>, <leftFootMicrosecs>, <rightHipMicrosecs>, <rightFootMicrosecs> }
//     milliseconds    - time to tween to to this keyframe's positions. E.g. 500 means it'll take 500ms to go from the
//                       robot's position at the start of this frame to the position specified in this frame
//     leftHipMicros   - position of left hip in servo microsecs.
//     leftFootMicros  - position of left hip in servo microsecs.
//     rightHipMicros  - position of left hip in servo microsecs.
//     rightFootMicros - position of left hip in servo microsecs.
// 
// The servo micro values, support a special value of -1. If this value is give, it tells
// the animation code to ignore this servo in this keyframe. i.e. That servo will
// stay in the position it had at the start of this keyframe.
//
// Also, the first element in the animation data arry is special. It is a metadata element.
// The first element is { <Num Frames>, 0, 0, 0, 0 }, which tells us the number of frames
// in the animation. So, the first actual keyframe is in animData[1], and the last keyframe
// is in animData[<Num Frames>]. (Where <Num Frames> is the value in animData[0][0].)
//----------------------------------------------------------------------------------

// Constants to make accessing the keyframe arrays more human readable.
const int TWEEN_TIME_VALUE = 0;
const int LEFT_HIP_VALUE   = 1;
const int LEFT_FOOT_VALUE  = 2;
const int RIGHT_HIP_VALUE  = 3;
const int RIGHT_FOOT_VALUE = 4;


// Constants used in the walking gait animation data.
const int FOOT_DELTA = 150;
const int HIP_DELTA  = FRONT_JOINT_HIPS * 120;


// Goes to the default standing straight position. Used by stopAnim().
int standStraightAnim[][5] = {
    // Metadata. First element is number of frames.
    { 1, 0, 0, 0, 0 },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }
};


// Prior to this, get the robot to Feet Flat, Feet Even (i.e. standStraightAnim).
int walkForwardAnim[][5] = {
    // Metadata. First element is number of frames.
    { 8, 0, 0, 0, 0 },
    
    // Tilt to left, Feet even
    { 300, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt to left, Right foot forward
    { 300, LeftHipIn(HIP_DELTA), LeftFootUp(FOOT_DELTA), RightHipOut(HIP_DELTA), RightFootDown(FOOT_DELTA) },
    
    // Feet flat, Right foot forward
    { 300, LeftHipIn(HIP_DELTA), LeftFootFlat(), RightHipOut(HIP_DELTA), RightFootFlat() },
    
    // Tilt to right, Right foot forward
    { 300, LeftHipIn(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipOut(HIP_DELTA), RightFootUp(FOOT_DELTA) },
    
    // Tilt to right, Feet even
    { 300, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt to right, Left foot forward
    { 300, LeftHipOut(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootUp(FOOT_DELTA) },
    
    // Feet flat, Left foot forward
    { 300, LeftHipOut(HIP_DELTA), LeftFootFlat(), RightHipIn(HIP_DELTA), RightFootFlat() },
    
    // Tilt to left, Left foot forward
    { 300, LeftHipOut(HIP_DELTA), LeftFootUp(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootDown(FOOT_DELTA) }
};


// Prior to this, get the robot to Feet Flat, Feet Even (i.e. standStraightAnim).
int walkBackwardAnim[][5] = {
    // Metadata. First element is number of frames.
    { 8, 0, 0, 0, 0 },
    
    // Tilt to left, Feet even
    { 300, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt to left, Left foot forward
    { 300, LeftHipOut(HIP_DELTA), LeftFootUp(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootDown(FOOT_DELTA) },

    // Feet flat, Left foot forward
    { 300, LeftHipOut(HIP_DELTA), LeftFootFlat(), RightHipIn(HIP_DELTA), RightFootFlat() },
        
    // Tilt to right, Left foot forward
    { 300, LeftHipOut(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootUp(FOOT_DELTA) },
    
    // Tilt to right, Feet even
    { 300, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt to right, Right foot forward
    { 300, LeftHipIn(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipOut(HIP_DELTA), RightFootUp(FOOT_DELTA) },
    
    // Feet flat, Right foot forward
    { 300, LeftHipIn(HIP_DELTA), LeftFootFlat(), RightHipOut(HIP_DELTA), RightFootFlat() },
    
    // Tilt to left, Right foot forward
    { 300, LeftHipIn(HIP_DELTA), LeftFootUp(FOOT_DELTA), RightHipOut(HIP_DELTA), RightFootDown(FOOT_DELTA) }
};

// Finish walk anim takes the robot from the end of walkForwardAnim/walkBackwardAnim back to standStraightAnim.
int walkEndAnim[][5] = {
    // Metadata. First element is number of frames.
    { 2, 0, 0, 0, 0 },
    
    // Tilt to left, Feet even
    { 300, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },

    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }
};


// Prior to this, get the robot to Feet Flat, Feet Even (i.e. standStraightAnim).
int turnLeftAnim[][5] = {
    // Metadata. First element is number of frames.
    { 6, 0, 0, 0, 0 },
    
    // Tilt to left, Feet even
    { 300, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt to left, Turn left hip, Turn right hip
    { 300, LeftHipIn(HIP_DELTA), LeftFootUp(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootDown(FOOT_DELTA) },

    // Feet flat, Turn left hip, Turn right hip
    { 300, LeftHipIn(HIP_DELTA), LeftFootFlat(), RightHipIn(HIP_DELTA), RightFootFlat() },
        
    // Tilt to right, Turn left hip, Turn right hip
    { 300, LeftHipIn(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootUp(FOOT_DELTA) },
    
    // Tilt to right, Feet even
    { 300, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }
};


// Prior to this, get the robot to Feet Flat, Feet Even (i.e. standStraightAnim).
int turnRightAnim[][5] = {
    // Metadata. First element is number of frames.
    { 6, 0, 0, 0, 0 },
    
    // Tilt to right, Feet even
    { 300, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt to right, Turn left hip, Turn right hip
    { 300, LeftHipIn(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootUp(FOOT_DELTA) },

    // Feet flat, Turn left hip, Turn right hip
    { 300, LeftHipIn(HIP_DELTA), LeftFootFlat(), RightHipIn(HIP_DELTA), RightFootFlat() },
        
    // Tilt to left, Turn left hip, Turn right hip
    { 300, LeftHipIn(HIP_DELTA), LeftFootUp(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootDown(FOOT_DELTA) },
    
    // Tilt to left, Feet even
    { 300, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }
};


// Shake head anim. Left right quickly to emulate shaking head.
int shakeHeadAnim[][5] = {
    // Metadata. First element is number of frames.
    { 4, 0, 0, 0, 0 },
    
    // Feet flat, Twist left
    { 150, LeftHipOut(HIP_DELTA), LeftFootFlat(), RightHipIn(HIP_DELTA), RightFootFlat() },
    
    // Feet flat, Feet even
    { 150, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() },
    
    // Feet flat, Twist right
    { 150, LeftHipIn(HIP_DELTA), LeftFootFlat(), RightHipOut(HIP_DELTA), RightFootFlat() },
    
    // Feet flat, Feet even
    { 150, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }    
};


// Wobble anim. Tilt left and right to do a fun wobble.
int wobbleAnim[][5] = {
    // Metadata. First element is number of frames.
    { 4, 0, 0, 0, 0 },
    
    // Tilt left, Feet even
    { 300, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() },
    
    // Tilt right, Feet even
    { 300, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }    
};

// Wobble left anim. Tilt left and back.
int wobbleLeftAnim[][5] = {
    // Metadata. First element is number of frames.
    { 2, 0, 0, 0, 0 },
    
    // Tilt left, Feet even
    { 300, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() },
};


// Wobble right anim. Tilt right and back.
int wobbleRightAnim[][5] = {
    // Metadata. First element is number of frames.
    { 2, 0, 0, 0, 0 },
    
    // Tilt right, Feet even
    { 300, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }    
};


// Tap feet anim. Tap both feet.
int tapFeetAnim[][5] = {
    // Metadata. First element is number of frames.
    { 2, 0, 0, 0, 0 },
    
    // Raise both feet, Feet even
    { 500, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 500, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() },
};


// Tap left foot anim.
int tapLeftFootAnim[][5] = {
    // Metadata. First element is number of frames.
    { 2, 0, 0, 0, 0 },
    
    // Raise left foot, Feet even
    { 500, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootFlat() },
    
    // Feet flat, Feet even
    { 500, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() },
};


// Tap right foot anim.
int tapRightFootAnim[][5] = {
    // Metadata. First element is number of frames.
    { 2, 0, 0, 0, 0 },
    
    // Raise right foot, Feet even
    { 500, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 500, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() },
};


// Bounce up and down anim.
int bounceAnim[][5] = {
    // Metadata. First element is number of frames.
    { 2, 0, 0, 0, 0 },
    
    // Raise both feet, Feet even
    { 500, LeftHipCentre(), LeftFootDown(300), RightHipCentre(), RightFootDown(300) },
    
    // Feet flat, Feet even
    { 500, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() },
};


// Shake Legs Animation.
int shakeLegsAnim[][5] = {
    // Metadata. First element is number of frames.
    { 14, 0, 0, 0, 0 },
    
    // Tilt left, Feet even
    { 300, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Right hip in
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Feet even
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Right hip out
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipOut(HIP_DELTA), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Feet even
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Right hip in
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Feet even
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() },
    
    // Tilt right, Feet even
    { 300, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Left hip in
    { 100, LeftHipIn(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Feet even
    { 100, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Left hip out
    { 100, LeftHipOut(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Feet even
    { 100, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }    
};


// Shake Left Leg Animation.
int shakeLeftLegAnim[][5] = {
    // Metadata. First element is number of frames.
    { 12, 0, 0, 0, 0 },
    
    // Tilt right, Feet even
    { 300, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Left hip in
    { 100, LeftHipIn(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Feet even
    { 100, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Left hip out
    { 100, LeftHipOut(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Feet even
    { 100, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Left hip in
    { 100, LeftHipIn(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Feet even
    { 100, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Left hip out
    { 100, LeftHipOut(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Feet even
    { 100, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Left hip in
    { 100, LeftHipIn(HIP_DELTA), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Tilt right, Feet even
    { 100, LeftHipCentre(), LeftFootDown(FOOT_DELTA), RightHipCentre(), RightFootUp(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }    
};


// Shake Right Leg Animation.
int shakeRightLegAnim[][5] = {
    // Metadata. First element is number of frames.
    { 12, 0, 0, 0, 0 },
    
    // Tilt left, Feet even
    { 300, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Right hip in
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Feet even
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Right hip out
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipOut(HIP_DELTA), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Feet even
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Right hip in
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Feet even
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Right hip out
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipOut(HIP_DELTA), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Feet even
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Right hip in
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipIn(HIP_DELTA), RightFootDown(FOOT_DELTA) },
    
    // Tilt left, Feet even
    { 100, LeftHipCentre(), LeftFootUp(FOOT_DELTA), RightHipCentre(), RightFootDown(FOOT_DELTA) },
    
    // Feet flat, Feet even
    { 300, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() },    
};


//----------------------------------------------------------------------------------
// Special dynamic animation data for setting/tweening servo positions.
//----------------------------------------------------------------------------------

// These are 2 special anim data that we use for the SetServos() function. They have
// a single frame. Those will change the data in these anim data and play them to 
// move the servos.
int setServosAnim1[][5] = {
    // Metadata. First element is number of frames.
    { 1, 0, 0, 0, 0 },
    
    // Tilt left, Feet even
    { 0, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }
};

int setServosAnim2[][5] = {
    // Metadata. First element is number of frames.
    { 1, 0, 0, 0, 0 },
    
    // Tilt left, Feet even
    { 0, LeftHipCentre(), LeftFootFlat(), RightHipCentre(), RightFootFlat() }
};


//----------------------------------------------------------------------------------
// Servo Variables
//----------------------------------------------------------------------------------

Servo servoLeftHip;
Servo servoLeftFoot;
Servo servoRightHip;
Servo servoRightFoot;

//----------------------------------------------------------------------------------
// State variables for playing animations.
//----------------------------------------------------------------------------------

// Milliseconds between animation updates.
const int millisBetweenAnimUpdate = 20;

// Time when we did the last animation update.
long timeAtLastAnimUpdate;

// Related to currently playing anim.
int  (*currAnim)[5];      // Current animation we're playing.
int  (*finishAnim)[5];    // Animation to play when the currAnim finishes or is stopped.
long timeAtStartOfFrame;  // millis() at last keyframe - frame we're lerping from
int  targetFrame;         // Frame we are lerping to
int  animNumLoops;        // Number of times to play the animation. -1 means loop forever.
char animCompleteStr[3] = "--"; // This is a 2 character string. When the anim is complete, 
                                // we print out the status as "<" + animComplereStr + ">".

// Related to anim queue. I.e. Next anim to play.
bool animInProgress;    // Whether an animation is playing

int  (*nextAnim)[5];      // This is the next animation to play once the current one is done. 
                          // i.e. It's like a queue of size 1!
                          // If curr is non-looping, we play this at the end of the current anim.
                          // If curr is looping, this starts at the end of the current loop, 
                          // replacing curr anim.
                          // If nothing is playing, this starts right away.
                          
int  (*nextFinishAnim)[5]; // This is the finish animation for the queued animation.

int  nextAnimNumLoops;    // Number of times to play the animation. -1 means loop forever.

char nextAnimCompleteStr[3] = "--"; // This is a 2 character string. When the anim is complete, 
                                    // we print out the status as "<" + animComplereStr + ">".

bool interruptInProgressAnim; // Whether to change anim immediately, interrupting the current one.


// Curr servo positions
int  currLeftHip;
int  currLeftFoot;
int  currRightHip;
int  currRightFoot;

// Servo positions at start of current keyframe
int  startLeftHip;
int  startLeftFoot;
int  startRightHip;
int  startRightFoot;


//-------------------------------------------------------------------------------
// Parser Variables
//-------------------------------------------------------------------------------

// Constant delimiter tag chars
const char START_CHAR = '<';
const char END_CHAR   = '>';
const char SEP_CHAR   = ',';

// Constants and a variable for the parser state.
const int PARSER_WAITING = 0; // Waiting for '<' to start parsing.
const int PARSER_COMMAND = 1; // Reading the command string.
const int PARSER_PARAM1  = 2; // Reading param 1.
const int PARSER_PARAM2  = 3; // Reading param 2.
const int PARSER_PARAM3  = 4; // Reading param 3.
const int PARSER_PARAM4  = 5; // Reading param 3.
const int PARSER_PARAM5  = 6; // Reading param 3.
const int PARSER_EXECUTE = 7; // Finished parsing a command, so execute it.

// Current parser state.
int currParserState = PARSER_WAITING; 

// String for storing the command. 2 chars for the command and 1 char for '\0'.
// We store the command here as we're parsing.
char currCmd[3] = "--";

// For tracking which letter we are in the command.
int currCmdIndex;

// Max command length.
const int CMD_LENGTH = 2;


// Current param values. Store them here after we parse them.
int currParam1Val;
int currParam2Val;
int currParam3Val;
int currParam4Val;
int currParam5Val;

// Variable for tracking which digit we're parsing in a param.
// We use this to convert the single digits back into a decimal value.
int currParamIndex;

// Whether the current param is negative.
boolean currParamNegative;

// Max parameter length. Stop parsing if it exceeds this.
const int MAX_PARAM_LENGTH = 6;


//===============================================================================
// Arduino setup() and loop().
//===============================================================================

void setup() 
{
    // Setup the main serial port
    softwareSerial.begin(SERIAL_SPEED);
    
    // Setup the Servos
    servoLeftHip.attach(  SERVO_LEFT_HIP,   LEFT_HIP_MIN,   LEFT_HIP_MAX);
    servoLeftFoot.attach( SERVO_LEFT_FOOT,  LEFT_FOOT_MIN,  LEFT_FOOT_MAX);
    servoRightHip.attach( SERVO_RIGHT_HIP,  RIGHT_HIP_MIN,  RIGHT_HIP_MAX);
    servoRightFoot.attach(SERVO_RIGHT_FOOT, RIGHT_FOOT_MIN, RIGHT_FOOT_MAX);

    // Set things up for the parser.
    setup_Parser();
    
    // Set things up for the animation code.
    setup_Animation();
}

void loop() 
{
    // Update the parser.
    loop_Parser();
    
    // Update the animation.
    loop_Animation();
}


//===============================================================================
// Related to the parser
//===============================================================================

// Sets up the parser stuff. Called in setup(). Should not be called elsewhere.
void setup_Parser()
{
    // Wait for first command.
    currParserState = PARSER_WAITING;
    
    // Print this response to say we've booted and are ready.
    softwareSerial.println("<OK>");
}


// Loop() for the parser stuff. Called in loop(). Should not be called elsewhere.
void loop_Parser()
{
    //---------------------------------------------------------
    // PARSER
    //
    // If there is data, parse it and process it.
    //---------------------------------------------------------
    
    // Read from pin serial port and write it out on USB port.
    if (softwareSerial.available() > 0)
    {
        char c = softwareSerial.read();
    
        // If we're in WAITING state, look for the START_CHAR.
        if (currParserState == PARSER_WAITING)
        {
            // If it's the START_CHAR, move out of this state...
            if (c == START_CHAR)
            {
                // Start parsing the command.
                currParserState = PARSER_COMMAND;
        
                // Reset thing ready for parsing
                currCmdIndex = 0;
                currCmd[0] = '-';
                currCmd[1] = '-';
                currParam1Val = 0;
                currParam2Val = 0;
                currParam3Val = 0;
                currParam4Val = 0;
                currParam5Val = 0;
            }
      
            // Otherwise, stay in this state.
        }
    
        // In the state to look for the command.
        else if (currParserState == PARSER_COMMAND)
        {
            // Else if it's a separator, parse parameter 1. But make sure it's not
            // empty, or else it's a parse error.
            if (c == SEP_CHAR)
            {
                if (currCmdIndex == CMD_LENGTH)
                {
                    currParserState = PARSER_PARAM1;
                    currParamIndex = 0;
                    currParamNegative = false;
                }
                else
                {
                    currParserState = PARSER_WAITING;
                }
            }
      
            // Else if it's the end char, there are no parameters, so we're ready to
            // process. But make sure it's not empty. Otherwise, it's a parse error.
            else if (c == END_CHAR)
            {
                if (currCmdIndex == CMD_LENGTH)
                {
                    currParserState = PARSER_EXECUTE;
                }
                else
                {
                    currParserState = PARSER_WAITING;
                }
            }
      
            // If we've got too many letters here, we have a parse error,
            // so abandon and go back to PARSER_WAITING
            else if ( (currCmdIndex >= CMD_LENGTH) || (c < 'A') || (c > 'Z') )
            {
                currParserState = PARSER_WAITING;
            }
      
            // Store the current character.
            else
            {
                currCmd[currCmdIndex] = c;
                currCmdIndex++;
            }
        }
    
        // In the state to parse param 1.
        else if (currParserState == PARSER_PARAM1)
        {
            // Else if it's a separator, parse parameter 1.
            if (c == SEP_CHAR)
            {
                if (currParamNegative)
                {
                    currParam1Val = -1 * currParam1Val;
                }

                currParserState = PARSER_PARAM2;
                currParamIndex = 0;
                currParamNegative = false;
            }
      
            // Else if it's the end char, there are no parameters, so we're ready to
            // process.
            else if (c == END_CHAR)
            {
                if (currParamNegative)
                {
                    currParam1Val = -1 * currParam1Val;
                }

                currParserState = PARSER_EXECUTE;
            }
      
            // Check for negative at the start.
            else if ( (currParamIndex == 0) && (c == '-') )
            {
                currParamNegative = true;
                currParamIndex++;
            }
            
            // If it's too long, or the character is not a digit, then it's
            // a parse error, so abandon and go back to PARSER_WAITING.
            else if ( (currParamIndex >= MAX_PARAM_LENGTH) || (c < '0') || (c > '9') )
            {
                currParserState = PARSER_WAITING;
            }

            // It's a valid character, so process it.
            else
            {
                // Shift existing value across and add new digit at the bottom.
                int currDigitVal = c - '0';
                currParam1Val = (currParam1Val * 10) + currDigitVal;
                currParamIndex++;
            }

        }
    
        // In the state to parse param 2.
        else if (currParserState == PARSER_PARAM2)
        {
            // Else if it's a separator, parse parameter 2.
            if (c == SEP_CHAR)
            {
                if (currParamNegative)
                {
                    currParam2Val = -1 * currParam2Val;
                }

                currParserState = PARSER_PARAM3;
                currParamIndex = 0;
                currParamNegative = false;
            }
      
            // Else if it's the end char, there are no parameters, so we're ready to
            // process.
            else if (c == END_CHAR)
            {
                if (currParamNegative)
                {
                    currParam2Val = -1 * currParam2Val;
                }

                currParserState = PARSER_EXECUTE;
            }
      
            // Check for negative at the start.
            else if ( (currParamIndex == 0) && (c == '-') )
            {
                currParamNegative = true;
                currParamIndex++;
            }
            
            // If it's too long, or the character is not a digit, then it's
            // a parse error, so abandon and go back to PARSER_WAITING.
            else if ( (currParamIndex >= MAX_PARAM_LENGTH) || (c < '0') || (c > '9') )
            {
                currParserState = PARSER_WAITING;
            }

            // It's a valid character, so process it.
            else
            {
                // Shift existing value across and add new digit at the bottom.
                int currDigitVal = c - '0';
                currParam2Val = (currParam2Val * 10) + currDigitVal;
                currParamIndex++;
            }

        }
    
        // In the state to parse param 3.
        else if (currParserState == PARSER_PARAM3)
        {
            // Else if it's a separator, parse parameter 2.
            if (c == SEP_CHAR)
            {
                if (currParamNegative)
                {
                    currParam3Val = -1 * currParam3Val;
                }

                currParserState = PARSER_PARAM4;
                currParamIndex = 0;
                currParamNegative = false;
            }
      
            // Else if it's the end char, there are no parameters, so we're ready to
            // process.
            else if (c == END_CHAR)
            {
                if (currParamNegative)
                {
                    currParam3Val = -1 * currParam3Val;
                }

                currParserState = PARSER_EXECUTE;
            }
      
            // Check for negative at the start.
            else if ( (currParamIndex == 0) && (c == '-') )
            {
                currParamNegative = true;
                currParamIndex++;
            }
            
            // If it's too long, or the character is not a digit, then it's
            // a parse error, so abandon and go back to PARSER_WAITING.
            else if ( (currParamIndex >= MAX_PARAM_LENGTH) || (c < '0') || (c > '9') )
            {
                currParserState = PARSER_WAITING;
            }

            // It's a valid character, so process it.
            else
            {
                // Shift existing value across and add new digit at the bottom.
                int currDigitVal = c - '0';
                currParam3Val = (currParam3Val * 10) + currDigitVal;
                currParamIndex++;
            }

        }
    
        // In the state to parse param 4.
        else if (currParserState == PARSER_PARAM4)
        {
            // Else if it's a separator, parse parameter 2.
            if (c == SEP_CHAR)
            {
                if (currParamNegative)
                {
                    currParam4Val = -1 * currParam4Val;
                }

                currParserState = PARSER_PARAM5;
                currParamIndex = 0;
                currParamNegative = false;
            }
      
            // Else if it's the end char, there are no parameters, so we're ready to
            // process.
            else if (c == END_CHAR)
            {
                if (currParamNegative)
                {
                    currParam4Val = -1 * currParam4Val;
                }

                currParserState = PARSER_EXECUTE;
            }
      
            // Check for negative at the start.
            else if ( (currParamIndex == 0) && (c == '-') )
            {
                currParamNegative = true;
                currParamIndex++;
            }
            
            // If it's too long, or the character is not a digit, then it's
            // a parse error, so abandon and go back to PARSER_WAITING.
            else if ( (currParamIndex >= MAX_PARAM_LENGTH) || (c < '0') || (c > '9') )
            {
                currParserState = PARSER_WAITING;
            }

            // It's a valid character, so process it.
            else
            {
                // Shift existing value across and add new digit at the bottom.
                int currDigitVal = c - '0';
                currParam4Val = (currParam4Val * 10) + currDigitVal;
                currParamIndex++;
            }

        }
            // In the state to parse param 5.
        else if (currParserState == PARSER_PARAM5)
        {
            // If it's the end char, there are no parameters, so we're ready to
            // process.
            if (c == END_CHAR)
            {
                if (currParamNegative)
                {
                    currParam5Val = -1 * currParam5Val;
                }
                currParserState = PARSER_EXECUTE;
            }
      
            // Check for negative at the start.
            else if ( (currParamIndex == 0) && (c == '-') )
            {
                currParamNegative = true;
                currParamIndex++;
            }
            
            // If it's too long, or the character is not a digit, then it's
            // a parse error, so abandon and go back to PARSER_WAITING.
            else if ( (currParamIndex >= MAX_PARAM_LENGTH) || (c < '0') || (c > '9') )
            {
                currParserState = PARSER_WAITING;
            }

            // It's a valid character, so process it.
            else
            {
                // Shift existing value across and add new digit at the bottom.
                int currDigitVal = c - '0';
                currParam5Val = (currParam5Val * 10) + currDigitVal;
                currParamIndex++;
            }

        }
    
        
        //---------------------------------------------------------
        // PARSER CODE HANDLER (Still part of Parser, but section that
        // processes completed commands)
        //
        // If the most recently read char completes a command,
        // then process the command, and clear the state to
        // go back to looking for a new command.
        //
        // The parsed items are stored in:
        //    currCmd, currParam1Val, currParam2Val, currParam3Val, 
        //             currParam4Val, currParam5Val
        //---------------------------------------------------------
    
        if (currParserState == PARSER_EXECUTE)
        {
            // Ready/OK Check: <OK>
            if ((currCmd[0] == 'O') && (currCmd[1] == 'K'))
            {
                softwareSerial.println("<OK>");
            }
            
            // Set Servo: <SV, time, leftHip, leftFoot, rightHip, rightFoot>
            // time      - time to tween to specified angles
            // leftHip   - microsecs from centre. -ve is hip in, +ve is hip out 
            // leftFoot  - microsecs from flat. -ve is foot down, +ve is foot up
            // rightHip  - microsecs from centre. -ve is hip in, +ve is hip out 
            // rightFoot - microsecs from flat. -ve is foot down, +ve is foot up
            else if ((currCmd[0] == 'S') && (currCmd[1] == 'V'))
            {
                int tweenTime = currParam1Val;
                if (currParam1Val < 0)
                {
                    tweenTime = 0;
                }
                SetServos(tweenTime, currParam2Val, currParam3Val, currParam4Val, currParam5Val, "SV");
            }
            
            // Stop/Reset: <ST>, Stops current anim. Also can be used to put robot into reset position.
            else if ((currCmd[0] == 'S') && (currCmd[1] == 'T'))
            {
                StopAnim("ST");
            }
            
            // Stop Immediate: <SI>
            else if ((currCmd[0] == 'S') && (currCmd[1] == 'I'))
            {
                StopAnimImmediate("SI");
            }
            
            // Forward: <FW, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'F') && (currCmd[1] == 'W'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(walkForwardAnim, walkEndAnim, numTimes, "FW");
            }
            
            // Backward: <BW, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'B') && (currCmd[1] == 'W'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(walkBackwardAnim, walkEndAnim, numTimes, "BW");
            }
            
            // Turn Left: <LT, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'L') && (currCmd[1] == 'T'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(turnLeftAnim, NULL, numTimes, "LT");
            }
            
            // Turn Right: <RT, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'R') && (currCmd[1] == 'T'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(turnRightAnim, NULL, numTimes, "RT");
            }
            
            // Shake Head: <SX, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'S') && (currCmd[1] == 'X'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(shakeHeadAnim, NULL, numTimes, "SX");
            }
            
            // Bounce: <BX, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'B') && (currCmd[1] == 'X'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(bounceAnim, NULL, numTimes, "BX");
            }
            
            // Wobble: <WX, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'W') && (currCmd[1] == 'X'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(wobbleAnim, NULL, numTimes, "WX");
            }
            
            // Wobble Left: <WY, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'W') && (currCmd[1] == 'Y'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(wobbleLeftAnim, NULL, numTimes, "WY");
            }
            
            // Wobble Right: <WZ, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'W') && (currCmd[1] == 'Z'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(wobbleRightAnim, NULL, numTimes, "WZ");
            }
            
            // Tap Feet: <TX, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'T') && (currCmd[1] == 'X'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(tapFeetAnim, NULL, numTimes, "TX");
            }
            
            // Tap Left Foot: <TY, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'T') && (currCmd[1] == 'Y'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(tapLeftFootAnim, NULL, numTimes, "TY");
            }
            
            // Tap Right Foot: <TZ, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'T') && (currCmd[1] == 'Z'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(tapRightFootAnim, NULL, numTimes, "TZ");
            }
            
            // Shake Legs: <LX, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'L') && (currCmd[1] == 'X'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(shakeLegsAnim, NULL, numTimes, "LX");
            }
            
            // Shake Left Leg: <LY, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'L') && (currCmd[1] == 'Y'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(shakeLeftLegAnim, NULL, numTimes, "LY");
            }
            
            // Shake Right Leg: <LZ, #Times>, -1 means continuous, 0 or no param is the same as 1 time.
            else if ((currCmd[0] == 'L') && (currCmd[1] == 'Z'))
            {
                int numTimes = currParam1Val;
                if (currParam1Val < 0)
                {
                    numTimes = -1;
                }
                
                PlayAnimNumTimes(shakeRightLegAnim, NULL, numTimes, "LZ");
            }
            
            //--------------------------------------------------
            // Clear the state and wait for the next command!
            // This must be done!
            //--------------------------------------------------
            currParserState = PARSER_WAITING;
        }
    }
}


//===============================================================================
// Related to playing servo animations.
//===============================================================================

// Call this to play the given animation once. Pass in NULL if there is no finishAnim.
void PlayAnim(int animToPlay[][5], int finishAnim[][5], char *completeStr)
{
    // Put this in the queue.
    PlayAnimNumTimes(animToPlay, finishAnim, 1, completeStr);
}

// Call this to loop the given animation. Pass in NULL if there is no finishAnim.
void LoopAnim(int animToPlay[][5], int finishAnim[][5], char *completeStr)
{
    // Put this in the queue.
    PlayAnimNumTimes(animToPlay, finishAnim, -1, completeStr);
}

// Call this to play the given animation the specified number of times. 
// -1 number of times will make it loop forever.
// Pass in NULL if there is no finishAnim.
void PlayAnimNumTimes(int animToPlay[][5], int finishAnim[][5], int numTimes, char *completeStr)
{
    // Put this in the queue.
    nextAnim         = animToPlay;
    nextFinishAnim   = finishAnim;
    nextAnimNumLoops = numTimes;

    // Save the completeStr
    if (completeStr == NULL)
    {
        nextAnimCompleteStr[0] = '-';
        nextAnimCompleteStr[1] = '-';
    }
    else
    {
        nextAnimCompleteStr[0] = completeStr[0];
        nextAnimCompleteStr[1] = completeStr[1];
    }
}

// Stop after the current animation.
void StopAnim(char *completeStr)
{
    // Put this in the queue.
    PlayAnimNumTimes(standStraightAnim, NULL, 1, completeStr);
}

// Stop immediately and lerp robot to zero position, interrupting 
// any animation that is in progress.
void StopAnimImmediate(char *completeStr)
{
    // Put this in the queue.
    interruptInProgressAnim = true;
    PlayAnimNumTimes(standStraightAnim, NULL, 1, completeStr);
}

// Moves servos to the specified positions. Time 0 will make it immediate. Otherwise,
// it'll tween it over a specified time.
// For positions, 0 means centered.
// For hips, -ve is hip left, +ve is hip right
// For feet, -ve is foot down, +ve is foot up
void SetServos(int tweenTime, int leftHip, int leftFoot, int rightHip, int rightFoot, char* completeStr)
{
    // Save the completeStr
    if (completeStr == NULL)
    {
        nextAnimCompleteStr[0] = '-';
        nextAnimCompleteStr[1] = '-';
    }
    else
    {
        nextAnimCompleteStr[0] = completeStr[0];
        nextAnimCompleteStr[1] = completeStr[1];
    }
    
    // Decide which tween data we use. We don't want to over-write the one that is
    // in progress. We have and reuse these to keep memory allocation fixed.
    int (*tweenServoData)[5];
    if (currAnim != setServosAnim1)
    {
        tweenServoData = setServosAnim1;
    }
    else
    {
        tweenServoData = setServosAnim2;
    }
    
    // Set the tween information into the animation data.
    tweenServoData[1][TWEEN_TIME_VALUE] = tweenTime;
    tweenServoData[1][LEFT_HIP_VALUE]   = LeftHipIn(leftHip);
    tweenServoData[1][LEFT_FOOT_VALUE]  = LeftFootUp(leftFoot);
    tweenServoData[1][RIGHT_HIP_VALUE]  = RightHipIn(rightHip);
    tweenServoData[1][RIGHT_FOOT_VALUE] = RightFootUp(rightFoot);
    
    // Queue this tween to be played next.
    PlayAnim(tweenServoData, NULL, completeStr);
}


// Set up variables for animation. This is called in setup(). Should be not called by anywhere else.
void setup_Animation()
{
    // Set the servos to the feet flat, feet even position.
    currLeftHip   = LEFT_HIP_CENTRE;
    currLeftFoot  = LEFT_FOOT_CENTRE;
    currRightHip  = RIGHT_HIP_CENTRE;
    currRightFoot = RIGHT_FOOT_CENTRE;
    UpdateServos();
    
    // Set the "start" positions to the current ones. So, when
    // we pay the next anim, we will tween from the current positions.
    startLeftHip   = currLeftHip;
    startLeftFoot  = currLeftFoot;
    startRightHip  = currRightHip;
    startRightFoot = currRightFoot;
    
    // No animation is playing yet, and nothing in the queue yet.
    timeAtLastAnimUpdate    = millis();
    animInProgress          = false;
    interruptInProgressAnim = false;
    currAnim       = NULL;
    finishAnim     = NULL;
    nextAnim       = NULL;
    nextFinishAnim = NULL;
}

// Loop function for processing animation. This is called in every loop(). Should be be called by anywhere else.
//
// NOTE: The way looping animations work is that they basically add themselves back to the queue
//       when a cycle is done, and if there's nothing already queued up! This way, looping animations
//       work in a similar way to single-play animations, and fits into the queueing system.
void loop_Animation()
{
    // Get the time at the start of this frame.
    long currTime = millis();

    //--------------------------------------------------------------------------------------
    // Decide if we want to perform the animation update. We don't execute this every frame.
    //--------------------------------------------------------------------------------------
    
    if (timeAtLastAnimUpdate + millisBetweenAnimUpdate > currTime)
    {
        // Not yet time to do an anim update, so jump out.
        return;
    }
    else
    {
        // We reset the timer, and then proceed below to handle the current anim update.
        timeAtLastAnimUpdate = currTime;
    }
    
    //--------------------------------------------------------------------------------------
    // Decide if we need to setup and start a new animation. We do if there's no anim 
    // playing or we've been asked to interrupt the anim.
    //--------------------------------------------------------------------------------------
    
    if ( (nextAnim != NULL) &&  (!animInProgress || interruptInProgressAnim) )
    {
        // If this was an interrupt, we also set the "start" servo positions
        // to the current ones. This way, the animation system will tween from the
        // current positions.
        if (interruptInProgressAnim)
        {
            // This is the place to notify someone of an animation finishing after getting interrupted
            // Print the command string we just finished. -1 parameter indicates it was interrupted.
            softwareSerial.print("<");
            softwareSerial.print(animCompleteStr);
            softwareSerial.println(",-1>");
            
            // Set the "start" positions to the current ones. So, when
            // we pay the next anim, we will tween from the current positions.
            startLeftHip   = currLeftHip;
            startLeftFoot  = currLeftFoot;
            startRightHip  = currRightHip;
            startRightFoot = currRightFoot;
            
            // We've handled any interrupt request, so clear the flag.
            interruptInProgressAnim = false;
        }
        
        // Store the animation we are now playing.
        currAnim           = nextAnim;
        finishAnim         = nextFinishAnim;
        animCompleteStr[0] = nextAnimCompleteStr[0];
        animCompleteStr[1] = nextAnimCompleteStr[1];

        nextAnim               = NULL; // Queue is cleared.
        nextFinishAnim         = NULL;
        nextAnimCompleteStr[0] = '-';
        nextAnimCompleteStr[1] = '-';
        
        // Record the number of times to play the animation.
        animNumLoops = nextAnimNumLoops;
        
        // Treat current time as start of frame for the initial lerp to the first frame.
        timeAtStartOfFrame = currTime;
        
        // Set the frame counters.
        targetFrame = 1; // First frame we are lerping to. Index 0 is metadata, so skip.
        
        // An animation is now in progress
        animInProgress = true;
    }

    //--------------------------------------------------------------------------------------
    // If we are currently playing an animation, then update the animation state and the
    // servo positions.
    //--------------------------------------------------------------------------------------
    
    if (animInProgress)
    {
        // Determine if we need to switch to the next frame.
        int timeInCurrFrame = currTime - timeAtStartOfFrame;
        if (timeInCurrFrame > currAnim[targetFrame][TWEEN_TIME_VALUE])
        {
            // Set the servo positions to the targetFrame's values.
            // We only set this if the value is > 0. -ve values means that
            // the current target keyframe did not alter that servos position.
            if (currAnim[targetFrame][LEFT_HIP_VALUE] >= 0)
            {
                currLeftHip = currAnim[targetFrame][LEFT_HIP_VALUE];
            }
            if (currAnim[targetFrame][LEFT_FOOT_VALUE] >= 0)
            {
                currLeftFoot = currAnim[targetFrame][LEFT_FOOT_VALUE];
            }
            if (currAnim[targetFrame][RIGHT_HIP_VALUE] >= 0)
            {
                currRightHip = currAnim[targetFrame][RIGHT_HIP_VALUE];
            }
            if (currAnim[targetFrame][RIGHT_FOOT_VALUE] >= 0)
            {
                currRightFoot = currAnim[targetFrame][RIGHT_FOOT_VALUE];
            }
            UpdateServos();
            
            // These current values are now the start of frame values.
            startLeftHip   = currLeftHip;
            startLeftFoot  = currLeftFoot;
            startRightHip  = currRightHip;
            startRightFoot = currRightFoot;
            
            // Now, we try to move to the next frame.
            // - If there is a next frame, set that as the new target, and proceed.
            // - If there's no next frame, but it's looping, we re-add this animation
            //   to the queue.
            // - If there's no next frame, and this is not looping, we stop animating.
            // (Remember that targetFrame is 1-based since the first element of the animation
            // data array is metadata)
            
            // Increment targetFrame, and reset time in the current frame.
            targetFrame++;
            timeAtStartOfFrame = currTime;
            
            // If there is no next frame, we stop this current animation.
            // If it is looping, then we re-queue the current animation if the queue is empty.
            if (targetFrame > NumOfFrames(currAnim))
            {
                // Stop the current animation.
                animInProgress = false;
                
                // If we're looping forever, and there's no next anim, re-queue the 
                // animation if the queue is empty.
                if ((animNumLoops < 0) && (nextAnim == NULL))
                {
                    LoopAnim(currAnim, finishAnim, animCompleteStr);
                }
                
                // If we're looping forever, and there is something in the queue, then
                // finish the animation and proceed.
                else if ((animNumLoops < 0) && (nextAnim != NULL))
                {
                    if (finishAnim != NULL)
                    {
                        // Switch to the finish anim.
                        currAnim       = finishAnim;
                        finishAnim     = NULL;
                        
                        // Record the number of times to play the animation.
                        animNumLoops = 1;
                        
                        // Treat current time as start of frame for the initial lerp to the first frame.
                        timeAtStartOfFrame = currTime;
                        
                        // Set the frame counters.
                        targetFrame = 1; // First frame we are lerping to. Index 0 is metadata, so skip.
                        
                        // An animation is now in progress
                        animInProgress = true;
                    }
                    else
                    {
                        // We've stopped, so can notify if needed.
                        // Print the command string we just finished.
                        softwareSerial.print("<");
                        softwareSerial.print(animCompleteStr);
                        softwareSerial.println(">");
                    }
                }
                
                // If we're looping a limited number of times, and there's no next anim,
                // re-queue the animation if the queue is empty.
                else if ((animNumLoops > 1) && (nextAnim == NULL))
                {
                    PlayAnimNumTimes(currAnim, finishAnim, animNumLoops-1, animCompleteStr);
                }
                
                // In this case, numAnimLoops is 1, this is the last loop through, so
                // we're done. We play the finishAnim first if needed.
                else
                {
                    // If there is a finish animation, switch to that animation.
                    if (finishAnim != NULL)
                    {
                        // Switch to the finish anim.
                        currAnim       = finishAnim;
                        finishAnim     = NULL;
                        
                        // Record the number of times to play the animation.
                        animNumLoops = 1;
                        
                        // Treat current time as start of frame for the initial lerp to the first frame.
                        timeAtStartOfFrame = currTime;
                        
                        // Set the frame counters.
                        targetFrame = 1; // First frame we are lerping to. Index 0 is metadata, so skip.
                        
                        // An animation is now in progress
                        animInProgress = true;
                    }
                    
                    // Otherwise, we're done! We've played the finishAnim if there was one.
                    else
                    {
                        // Print the command string we just finished.
                        softwareSerial.print("<");
                        softwareSerial.print(animCompleteStr);
                        softwareSerial.println(">");
                    }
                }
            }
        }
        
        // If we're still animating (i.e. the previous check didn't find that
        // we've finished the current animation), then proceed.
        if (animInProgress)
        {
            // Set the servos per data in the current frame. We only update the servos that have target
            // microsecond values > 0. This is to support the feature where we leave a servo at its
            // existing position if an animation data item is -1.
            float frameTimeFraction = (currTime - timeAtStartOfFrame) / ((float) currAnim[targetFrame][TWEEN_TIME_VALUE]);
            
            if (currAnim[targetFrame][LEFT_HIP_VALUE] >= 0)
            {
                currLeftHip = startLeftHip + ((currAnim[targetFrame][LEFT_HIP_VALUE] - startLeftHip) * frameTimeFraction);
            }
            
            if (currAnim[targetFrame][LEFT_FOOT_VALUE] >= 0)
            {
                currLeftFoot = startLeftFoot + ((currAnim[targetFrame][LEFT_FOOT_VALUE] - startLeftFoot)  * frameTimeFraction);
            }
            
            if (currAnim[targetFrame][RIGHT_HIP_VALUE] >= 0)
            {
                currRightHip = startRightHip  + ((currAnim[targetFrame][RIGHT_HIP_VALUE] - startRightHip) * frameTimeFraction);
            }
            
            if (currAnim[targetFrame][RIGHT_FOOT_VALUE] >= 0)
            {
                currRightFoot = startRightFoot + ((currAnim[targetFrame][RIGHT_FOOT_VALUE] - startRightFoot) * frameTimeFraction);
            }
            
            UpdateServos();
        }
    }
}


// Move all the servo to the positions set in the curr... variables.
// In the code, we update those variables and then call this to set the servos.
void UpdateServos()
{
    servoLeftHip.writeMicroseconds(currLeftHip);
    servoLeftFoot.writeMicroseconds(currLeftFoot);
    servoRightHip.writeMicroseconds(currRightHip);
    servoRightFoot.writeMicroseconds(currRightFoot);
}


// Return the number of frames in the given animation data.
// Have this helper function to avoid the "magic number" reference of animData[0][0].
int NumOfFrames(int animData[][5])
{
    return animData[0][0];
}


//------------------------------------------------------------------------------
// Helper functions to help calculate joint values in a more user-friendly way.
// You can adjust the signs here if the servos are setup in a different way.
// Updating here means the animation data doesn't need to be modified if the
// servos are setup differently.
// (E.g. Original Bob's hip servos are backwards to MobBob's.)
//
// (Also, I find it hard to remember the signs to use for each servo since they 
// are different for left/right hips, and for left/right feet.)
//------------------------------------------------------------------------------


int LeftHipCentre()              { return LEFT_HIP_CENTRE; }
int LeftHipIn(int millisecs)     { return LEFT_HIP_CENTRE + (FRONT_JOINT_HIPS * millisecs); }
int LeftHipOut(int millisecs)    { return LEFT_HIP_CENTRE - (FRONT_JOINT_HIPS * millisecs); }

int RightHipCentre()             { return RIGHT_HIP_CENTRE; }
int RightHipIn(int millisecs)    { return RIGHT_HIP_CENTRE - (FRONT_JOINT_HIPS * millisecs); }
int RightHipOut(int millisecs)   { return RIGHT_HIP_CENTRE + (FRONT_JOINT_HIPS * millisecs); }

int LeftFootFlat()               { return LEFT_FOOT_CENTRE; }
int LeftFootUp(int millisecs)    { return LEFT_FOOT_CENTRE - millisecs; }
int LeftFootDown(int millisecs)  { return LEFT_FOOT_CENTRE + millisecs; }

int RightFootFlat()              { return RIGHT_FOOT_CENTRE; }
int RightFootUp(int millisecs)   { return RIGHT_FOOT_CENTRE + millisecs; }
int RightFootDown(int millisecs) { return RIGHT_FOOT_CENTRE - millisecs; }




