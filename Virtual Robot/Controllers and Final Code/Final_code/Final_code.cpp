// File:          ChessBoardExplore.cpp
// Date:          2023/02/17
// Description:   Traverse the World

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include <iostream>
#include <vector>

#define GRIPPER_MOTOR_MAX_SPEED 0.1
#define TIMESTEP 16

#define MAX_SPEED 12.28
#define MID_SPEED 6.28
#define TURN_SPEED 6.1
#define THRESHOLD 300

#define OBSTACLE_THRESHOLD 30
#define OBSTACLE_THRESHOLD1 40

using namespace webots;

void turnRight();
void turnLeft();
void advanceTile(double s = 1.0);
void advanceTileBack(double s = 1.0);
void travelMaze();
bool checkPiece();
void dropTheBox();
void exitChessboard();

void wallFollowing();

void dottedLineFollowing();
void startLineFollowing();
void goForwardAndPickUp();

void chamberExplore();

Robot* robot;

Motor* rightMotor;
Motor* leftMotor;

Motor* gripperLift;

Motor* rightFinger;
Motor* leftFinger;

DistanceSensor* frontSensor;
DistanceSensor* leftSensor;
DistanceSensor* rightSensor;

DistanceSensor* frontDSLaser;
DistanceSensor* topDS;
DistanceSensor* frontIR;

DistanceSensor* ir_2;
DistanceSensor* ir_1;
DistanceSensor* ir_3;
DistanceSensor* ir_4;
DistanceSensor* ir_5;
DistanceSensor* ir_6;
DistanceSensor* ir_7;

int main(int argc, char **argv) {
    // create the Robot instance.
    robot = new Robot();

    int timeStep = TIMESTEP;

    rightMotor = robot->getMotor("rightWheel");
    leftMotor = robot->getMotor("leftWheel");

    gripperLift = robot->getMotor("lift motor");

    rightFinger = robot->getMotor("right finger motor");
    leftFinger = robot->getMotor("left finger motor");

    frontDSLaser = robot->getDistanceSensor("frontDSLaser");
    frontDSLaser->enable(timeStep);

    topDS = robot->getDistanceSensor("topDS");
    topDS->enable(timeStep);

    frontIR = robot->getDistanceSensor("frontIR");
    frontIR->enable(timeStep);

    ir_1 = robot->getDistanceSensor("ir1");
    ir_2 = robot->getDistanceSensor("ir2");
    ir_3 = robot->getDistanceSensor("ir3");
    ir_4 = robot->getDistanceSensor("ir4");
    ir_5 = robot->getDistanceSensor("ir5");
    ir_6 = robot->getDistanceSensor("ir6");
    ir_7 = robot->getDistanceSensor("ir7");

    ir_1->enable(TIMESTEP);
    ir_2->enable(TIMESTEP);
    ir_3->enable(TIMESTEP);
    ir_4->enable(TIMESTEP);
    ir_5->enable(TIMESTEP);
    ir_6->enable(TIMESTEP);
    ir_7->enable(TIMESTEP);

    startLineFollowing();
    wallFollowing();
    dottedLineFollowing();
    travelMaze();
    exitChessboard();
    advanceTile(0.5);
    chamberExplore();
    startLineFollowing();

    delete robot;
    return 0;
}

void travelMaze() {
    //pickUpTheBox();
    goForwardAndPickUp();
    advanceTile(0.1);

    int advanceStraightCount = 0;
    bool kingFound = false;

    while (true) {
        turnRight();

        double newFrontDSValue = frontDSLaser->getValue();
        std::cout << "FrontDS: " << newFrontDSValue << "\n";

        // Check for obstacle on right
        if (newFrontDSValue > 900) {
            turnLeft();
        }
        else {
            std::cout << "OBSTACLE FOUND\n";

            newFrontDSValue = frontDSLaser->getValue();
            std::cout << newFrontDSValue << "\n";

            // Go to the right chess piece
            int advanceRightCount = 0;
            while (newFrontDSValue > 120) {
                advanceTile();
                advanceRightCount++;

                newFrontDSValue = frontDSLaser->getValue();
            }

            std::cout << "CHECKING THE PIECE\n";
            kingFound = checkPiece();

            // Come back to the main path
            for (int i = 0; i < advanceRightCount; i++) {
                advanceTileBack();
            }

            turnLeft();
        }

        //  Check for obstacle on left
        if (!kingFound) {
            turnLeft();
            newFrontDSValue = frontDSLaser->getValue();
            std::cout << "FrontDS Left: " << newFrontDSValue << "\n";

            if (newFrontDSValue > 100) {
                turnRight();
            }
            else {
                std::cout << "OBSTACLE FOUND\n";
                std::cout << "Checking the Piece\n";
                kingFound = checkPiece();
                turnRight();
            }
        }

        if (kingFound) {
            advanceTileBack();
            advanceTile(0.6);
            dropTheBox();
            advanceTileBack(0.6);
            advanceStraightCount--;
            break;
        }

        newFrontDSValue = frontDSLaser->getValue();
        if (newFrontDSValue > 100 && advanceStraightCount != 7) {
            advanceTile();
            advanceStraightCount++;
        }
        else {
            break;
        }
    }

    // 180 Turn and go back to A7
    //turnRight();
    //turnRight();
    for (int i = 0; i < advanceStraightCount; i++) {
        advanceTileBack();
    }


    // Check Left Column
    turnLeft();
    robot->step(TIMESTEP);
    bool canGoLeft = frontDSLaser->getValue() > 100;
    bool shouldTurnLeft = false;
    
    if (!kingFound && canGoLeft) {
        advanceTile();

        turnRight();

        double newFrontDSValue = frontDSLaser->getValue();
        std::cout << "FrontDS value: " << newFrontDSValue << "\n";

        if (newFrontDSValue > 950) {
            turnLeft();
        }
        else {
            std::cout << "OBSTACLE FOUND\n";

            newFrontDSValue = frontDSLaser->getValue();
            std::cout << "FrontDS: " << newFrontDSValue << "\n";

            int advanceRightCount = 0;
            while (newFrontDSValue > 120) {
                advanceTile();
                advanceRightCount++;

                newFrontDSValue = frontDSLaser->getValue();
            }

            std::cout << "Checking the piece\n";
            kingFound = checkPiece();

            if (kingFound) shouldTurnLeft = true;

            // Come back to the main path
            for (int i = 0; i < advanceRightCount; i++) {
                advanceTileBack();
            }

            turnLeft();
        }

        if (kingFound) {
            advanceTileBack();
            advanceTile(0.6);
            dropTheBox();
            advanceTileBack(0.6);
        }
        else {
            advanceTileBack();
        }

        turnRight();
    }

    if (!canGoLeft && !kingFound) turnRight();

    turnRight();
    robot->step(TIMESTEP);
    bool canGoRight = frontDSLaser->getValue() > 100;

    if (!kingFound && canGoRight) {

        advanceTile();
        advanceStraightCount = 1;

        while (true) {
            turnLeft();

            double newFrontDSValue = frontDSLaser->getValue();
            std::cout << "FrontDS value: " << newFrontDSValue << "\n";

            // Check for obstacle on right
            if (newFrontDSValue > 950) {
                turnRight();
            }
            else {
                std::cout << "OBSTACLE FOUND\n";

                newFrontDSValue = frontDSLaser->getValue();
                std::cout << "FrontDS: " << newFrontDSValue << "\n";

                // Go to the right chess piece
                int advanceRightCount = 0;
                while (newFrontDSValue > 120) {
                    advanceTile();
                    advanceRightCount++;

                    newFrontDSValue = frontDSLaser->getValue();
                }

                std::cout << "CHECKING THE PIECE\n";
                kingFound = checkPiece();

                // Come back to the main path
                for (int i = 0; i < advanceRightCount; i++) {
                    advanceTileBack();
                }

                turnRight();
            }

            if (kingFound) {
                advanceTileBack();
                advanceTile(0.6);
                dropTheBox();
                advanceTileBack(0.6);
                advanceStraightCount--;
                break;
            }

            newFrontDSValue = frontDSLaser->getValue();
            if (newFrontDSValue > 100) {
                advanceTile();
                advanceStraightCount++;
            }
            else {
                break;
            }
        }

        // 180 Turn and go back to A7
        //turnRight();
        //turnRight();
        for (int i = 0; i < advanceStraightCount; i++) {
            advanceTileBack();
        }

        turnLeft();
    }

    if (shouldTurnLeft) turnLeft();
}

bool checkPiece() {
    double topDSValue = topDS->getValue();
    std::cout << "TOP DS: " << topDSValue << "\n";

    double frontIRValue = frontIR->getValue();
    std::cout << "FRONT IR: " << frontIRValue << "\n";

    if (topDSValue < 500 && frontIRValue < 500) {
        std::cout << "WHITE KING FOUND\n";

        return true;
    }

    return false;
}

void turnRight() {
    rightMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);
    rightMotor->setVelocity(5.0);
    leftMotor->setVelocity(-5.0);

    double elapsedTime = 0;

    while (elapsedTime < 109) {
        robot->step(TIMESTEP);

        //double newFrontDSValue = frontDS->getValue();

        //cout << newFrontDSValue << "\n";
        //std::cout << elapsedTime << "\n";
        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);

    std::cout << "TURNED RIGHT\n";
}

void turnLeft() {
    rightMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);
    rightMotor->setVelocity(-5.0);
    leftMotor->setVelocity(5.0);

    double elapsedTime = 0;

    while (elapsedTime < 109) {
        robot->step(TIMESTEP);

        //double newFrontDSValue = frontDS->getValue();

        //cout << newFrontDSValue << "\n";

        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);

    std::cout << "TURNED LEFT\n";
}

void advanceTile(double s) {
    double elapsedTime = 0;

    rightMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);

    rightMotor->setVelocity(-5.0);
    leftMotor->setVelocity(-5.0);

    while (elapsedTime < 295 * s) {
        robot->step(TIMESTEP);
        //std::cout << elapsedTime << "\n";
        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);
}

void advanceTileBack(double s) {
    double elapsedTime = 0;

    rightMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);

    rightMotor->setVelocity(5.0);
    leftMotor->setVelocity(5.0);

    while (elapsedTime < 295 * s) {
        robot->step(TIMESTEP);
        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);
}

void dropTheBox() {
    gripperLift->setVelocity(0.1);
    rightFinger->setVelocity(0.1);
    leftFinger->setVelocity(0.1);

    gripperLift->setPosition(0.13);
    rightFinger->setPosition(0.1);
    leftFinger->setPosition(0.1);
}

void exitChessboard() {
    turnRight();
    advanceTile();
    turnRight();
    advanceTile(0.8);
    turnLeft();

    for (int i = 0; i < 5; i++) advanceTile();

    advanceTile(0.93);

    turnLeft();

    for (int i = 0; i < 9; i++) advanceTile();
}

void dottedLineFollowing() {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    double ir_val[7] = { 0,0,0,0,0,0,0 };
    double ir_weight[7] = { -20,-10,-5,0,5,10,20 };

    // initiate PID controller
    double error = 0;
    double previousError = 0;
    double totalError = 0;
    double k_p = 0.5;
    double k_i = 0.000001;
    double k_d = 0.0001;

    double red = 300;
    double black = 500;

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(TIMESTEP) != -1) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();
        //double ds_left_value = ds_left->getValue();
        //double ds_right_value = ds_right->getValue();
        //double ds_front_value = ds_front->getValue();

        //read values from ir sensors
        double ir_1_value = ir_1->getValue();
        double ir_2_value = ir_2->getValue();
        double ir_3_value = ir_3->getValue();
        double ir_4_value = ir_4->getValue();
        double ir_5_value = ir_5->getValue();
        double ir_6_value = ir_6->getValue();
        double ir_7_value = ir_7->getValue();

        std::cout << ir_1_value << ", " << ir_2_value << ", " << ir_3_value << ", " << ir_4_value << ", " << ir_5_value << ", " << ir_6_value << ", " << ir_7_value << "\n";


        if ((ir_1_value<red && ir_7_value>red && ir_7_value < black) || (ir_7_value<red && ir_1_value>red && ir_1_value < black))
        {
            if (ir_1_value > red && ir_1_value < THRESHOLD)
            {
                robot->step(1000);
                leftMotor->setVelocity(MID_SPEED);
                rightMotor->setVelocity(0.1);

                robot->step(1300);
            }

            if (ir_7_value > red && ir_7_value < THRESHOLD)

            {
                robot->step(1000);
                leftMotor->setVelocity(0.1);
                rightMotor->setVelocity(MID_SPEED);

                robot->step(1300);
            }
        }

        if (ir_1_value > THRESHOLD) { ir_val[0] = 1; }
        else { ir_val[0] = 0; }

        if (ir_2_value > THRESHOLD) { ir_val[1] = 1; }
        else { ir_val[1] = 0; }

        if (ir_3_value > THRESHOLD) { ir_val[2] = 1; }
        else { ir_val[2] = 0; }

        if (ir_4_value > THRESHOLD) { ir_val[3] = 1; }
        else { ir_val[3] = 0; }

        if (ir_5_value > THRESHOLD) { ir_val[4] = 1; }
        else { ir_val[4] = 0; }

        if (ir_6_value > THRESHOLD) { ir_val[5] = 1; }
        else { ir_val[5] = 0; }

        if (ir_7_value > THRESHOLD) { ir_val[6] = 1; }
        else { ir_val[6] = 0; }

        //std::cout<<"ir_1 :"<<ir_1_value<<std::endl;
        std::cout << "ir_1 :" << ir_val[0] << std::endl;
        //std::cout<<"ir_2 :"<<ir_2_value<<std::endl;
        std::cout << "ir_2 :" << ir_val[1] << std::endl;
        //std::cout<<"ir_3 :"<<ir_3_value<<std::endl;
        std::cout << "ir_3 :" << ir_val[2] << std::endl;
        //std::cout<<"ir_4 :"<<ir_4_value<<std::endl;
        std::cout << "ir_4 :" << ir_val[3] << std::endl;
        //std::cout<<"ir_5 :"<<ir_5_value<<std::endl;
        std::cout << "ir_5 :" << ir_val[4] << std::endl;
        //std::cout<<"ir_6 :"<<ir_6_value<<std::endl;
        std::cout << "ir_6 :" << ir_val[5] << std::endl;
        //std::cout<<"ir_7 :"<<ir_6_value<<std::endl;
        std::cout << "ir_7 :" << ir_val[6] << std::endl;


        if (ir_val[0] == 1 && ir_val[1] == 1 && ir_val[2] == 0 && ir_val[3] == 0 && ir_val[4] == 0 && ir_val[5] == 0 && ir_val[6] == 0)
        {
            robot->step(1000);
            leftMotor->setVelocity(0.1);
            rightMotor->setVelocity(TURN_SPEED);

            robot->step(1300);

        }

        else if (ir_val[0] == 0 && ir_val[1] == 0 && ir_val[2] == 0 && ir_val[3] == 0 && ir_val[4] == 0 && ir_val[5] == 1 && ir_val[6] == 1)
        {
            robot->step(1000);
            leftMotor->setVelocity(TURN_SPEED);
            rightMotor->setVelocity(0.1);

            robot->step(1300);

        }





        else
        {
            double error = 0;

            for (int i = 0; i < 7; i++)
            {
                error += ir_val[i] * ir_weight[i];
                //std::cout<<"error "<<error<<std::endl;
            }

            totalError += error;

            //std::cout<<"total error "<<totalError<<std::endl;
            double d = error - previousError;

            //double PID_val = (error*k_p + d*k_d + totalError*k_i);
            double PID_val = (error * k_p + d * k_d + totalError * k_i);
            //std::cout<<"error "<<error<<std::endl; 
            //std::cout<<"PID_val "<<PID_val<<std::endl;


            //leftMotor->setVelocity(-MID_SPEED);
            //right_motor->setVelocity(-MID_SPEED);

            double right_motor_speed = MID_SPEED + PID_val;
            double left_motor_speed = MID_SPEED - PID_val;


            if (right_motor_speed < 0) { right_motor_speed = -0.1; }
            if (right_motor_speed > MAX_SPEED) { right_motor_speed = MAX_SPEED; }

            if (left_motor_speed < 0) { left_motor_speed = -0.1; }
            if (left_motor_speed > MAX_SPEED) { left_motor_speed = MAX_SPEED; }

            //std::cout<<"right_motor"<<right_motor_speed<<std::endl;
            //std::cout<<"left_motor"<<left_motor_speed<<std::endl;

            // Process sensor data here.

            // Enter here functions to send actuator commands, like:
            //  motor->setPosition(10.0);




            leftMotor->setVelocity(-left_motor_speed);
            rightMotor->setVelocity(-right_motor_speed);
            previousError = error;
            std::cout << "right_motor" << right_motor_speed << std::endl;
            std::cout << "left_motor" << left_motor_speed << std::endl;




            //print values
            //std::cout<<"Distance sensor left :"<<ds_left_value<<std::endl;
            //std::cout<<"Distance sensor right :"<<ds_right_value<<std::endl;
            //std::cout<<"Distance sensor front :"<<ds_front_value<<std::endl; 

            /*//std::cout<<"ir_1 :"<<ir_1_value<<std::endl;
            std::cout<<"ir_1 :"<<ir_val[0]<<std::endl;
            //std::cout<<"ir_2 :"<<ir_2_value<<std::endl;
            std::cout<<"ir_2 :"<<ir_val[1]<<std::endl;
            //std::cout<<"ir_3 :"<<ir_3_value<<std::endl;
            std::cout<<"ir_3 :"<<ir_val[2]<<std::endl;
            //std::cout<<"ir_4 :"<<ir_4_value<<std::endl;
            std::cout<<"ir_4 :"<<ir_val[3]<<std::endl;
            //std::cout<<"ir_5 :"<<ir_5_value<<std::endl;
            std::cout<<"ir_5 :"<<ir_val[4]<<std::endl;
            //std::cout<<"ir_6 :"<<ir_6_value<<std::endl;
            std::cout<<"ir_6 :"<<ir_val[5]<<std::endl;
            //std::cout<<"ir_7 :"<<ir_6_value<<std::endl;
            std::cout<<"ir_7 :"<<ir_val[5]<<std::endl;*/
        }

        if (ir_1_value > 350 && ir_1_value < 400 &&
            ir_2_value > 350 && ir_2_value < 400 &&
            ir_3_value > 350 && ir_3_value < 400 &&
            ir_4_value > 350 && ir_4_value < 400 &&
            ir_5_value > 350 && ir_5_value < 400 &&
            ir_6_value > 350 && ir_6_value < 400 &&
            ir_7_value > 350 && ir_7_value < 400) {

            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);

            leftMotor->setPosition(0.0);
            rightMotor->setPosition(0.0);

            break;
        }

    };
}

void goForwardAndPickUp() {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(-5.0);
    rightMotor->setVelocity(-5.0);

    gripperLift->setVelocity(0.03);
    rightFinger->setVelocity(0.1);
    leftFinger->setVelocity(0.1);

    gripperLift->setPosition(0.13);
    rightFinger->setPosition(0.12);
    leftFinger->setPosition(0.12);

    robot->step(TIMESTEP);
    double newFrontDSValue = frontDSLaser->getValue();

    bool pickedUp = false;

    while (true) {
        robot->step(TIMESTEP);
        std::cout << newFrontDSValue << '\n';

        if (newFrontDSValue < 10) {
            rightFinger->setPosition(0.06);
            leftFinger->setPosition(0.06);
            gripperLift->setPosition(-0.01);

            pickedUp = true;
        }

        if (pickedUp && newFrontDSValue > 10) {
            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);

            break;
        }

        newFrontDSValue = frontDSLaser->getValue();
    }
}

void wallFollowing() {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    // initialize sensors
    frontSensor = robot->getDistanceSensor("frontDS");
    leftSensor = robot->getDistanceSensor("leftFrontDS");
    rightSensor = robot->getDistanceSensor("rightFrontDS");

    //DistanceSensor *leftSensorB = robot->getDistanceSensor("leftBackDS");
   // DistanceSensor *rightSensorB = robot->getDistanceSensor("rightBackDS");
    frontSensor->enable(TIMESTEP);
    leftSensor->enable(TIMESTEP);
    rightSensor->enable(TIMESTEP);
    //leftSensorB->enable(TIMESTEP);
    //rightSensorB->enable(TIMESTEP);

    while (robot->step(TIMESTEP) != -1) {

        //read values from ir sensors
        double ir_1_value = ir_1->getValue();
        double ir_2_value = ir_2->getValue();
        double ir_3_value = ir_3->getValue();
        double ir_4_value = ir_4->getValue();
        double ir_5_value = ir_5->getValue();
        double ir_6_value = ir_6->getValue();
        double ir_7_value = ir_7->getValue();

        if (ir_1_value < 400 &&
            ir_2_value < 400 &&
            ir_3_value < 400 &&
            ir_4_value < 400 &&
            ir_5_value < 400 &&
            ir_6_value < 400 &&
            ir_7_value < 400) {

            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);

            leftMotor->setPosition(0.0);
            rightMotor->setPosition(0.0);

            break;
        }


        double frontReading = frontSensor->getValue();
        double leftReading = leftSensor->getValue();
        double rightReading = rightSensor->getValue();
        // double leftReadingB = leftSensorB->getValue();
        // double rightReadingB = rightSensorB->getValue();
        std::cout << "0" << std::endl;
        // if there is an obstacle in front, turn left
        if (frontReading > 0 && frontReading < 40) {
            leftMotor->setVelocity(MAX_SPEED);
            rightMotor->setVelocity(-MAX_SPEED);
            std::cout << "1" << std::endl;
        }
        // if there is an obstacle on the left, turn right
        else if (leftReading > 0 && leftReading < OBSTACLE_THRESHOLD1) {
            leftMotor->setVelocity(-MAX_SPEED);
            rightMotor->setVelocity(2);

        }
        // if there is an obstacle on the right, turn left
        else if (rightReading > 0 && rightReading < OBSTACLE_THRESHOLD) {
            leftMotor->setVelocity(2);
            rightMotor->setVelocity(-MAX_SPEED);

        }
        // if there are no obstacles, go straight
        else {
            leftMotor->setVelocity(-MAX_SPEED);
            rightMotor->setVelocity(-MAX_SPEED);

        }
    }
}

void startLineFollowing() {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    double ir_val[7] = { 0,0,0,0,0,0,0 };
    double ir_weight[7] = { -20,-10,-5,0,5,10,20 };

    // initiate PID controller
    double error = 0;
    double previousError = 0;
    double totalError = 0;
    double k_p = 0.5;
    double k_i = 0.000001;
    double k_d = 0.0001;

    double red = 300;
    double black = 500;

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(TIMESTEP) != -1) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();
        //double ds_left_value = ds_left->getValue();
        //double ds_right_value = ds_right->getValue();
        //double ds_front_value = ds_front->getValue();

        //read values from ir sensors
        double ir_1_value = ir_1->getValue();
        double ir_2_value = ir_2->getValue();
        double ir_3_value = ir_3->getValue();
        double ir_4_value = ir_4->getValue();
        double ir_5_value = ir_5->getValue();
        double ir_6_value = ir_6->getValue();
        double ir_7_value = ir_7->getValue();

        std::cout << ir_1_value << ", " << ir_2_value << ", " << ir_3_value << ", " << ir_4_value << ", " << ir_5_value << ", " << ir_6_value << ", " << ir_7_value << "\n";


        if ((ir_1_value<red && ir_7_value>red && ir_7_value < black) || (ir_7_value<red && ir_1_value>red && ir_1_value < black))
        {
            if (ir_1_value > red && ir_1_value < THRESHOLD)
            {
                robot->step(1000);
                leftMotor->setVelocity(MID_SPEED);
                rightMotor->setVelocity(0.1);

                robot->step(1300);
            }

            if (ir_7_value > red && ir_7_value < THRESHOLD)

            {
                robot->step(1000);
                leftMotor->setVelocity(0.1);
                rightMotor->setVelocity(MID_SPEED);

                robot->step(1300);
            }
        }

        if (ir_1_value > THRESHOLD) { ir_val[0] = 1; }
        else { ir_val[0] = 0; }

        if (ir_2_value > THRESHOLD) { ir_val[1] = 1; }
        else { ir_val[1] = 0; }

        if (ir_3_value > THRESHOLD) { ir_val[2] = 1; }
        else { ir_val[2] = 0; }

        if (ir_4_value > THRESHOLD) { ir_val[3] = 1; }
        else { ir_val[3] = 0; }

        if (ir_5_value > THRESHOLD) { ir_val[4] = 1; }
        else { ir_val[4] = 0; }

        if (ir_6_value > THRESHOLD) { ir_val[5] = 1; }
        else { ir_val[5] = 0; }

        if (ir_7_value > THRESHOLD) { ir_val[6] = 1; }
        else { ir_val[6] = 0; }

        //std::cout<<"ir_1 :"<<ir_1_value<<std::endl;
        std::cout << "ir_1 :" << ir_val[0] << std::endl;
        //std::cout<<"ir_2 :"<<ir_2_value<<std::endl;
        std::cout << "ir_2 :" << ir_val[1] << std::endl;
        //std::cout<<"ir_3 :"<<ir_3_value<<std::endl;
        std::cout << "ir_3 :" << ir_val[2] << std::endl;
        //std::cout<<"ir_4 :"<<ir_4_value<<std::endl;
        std::cout << "ir_4 :" << ir_val[3] << std::endl;
        //std::cout<<"ir_5 :"<<ir_5_value<<std::endl;
        std::cout << "ir_5 :" << ir_val[4] << std::endl;
        //std::cout<<"ir_6 :"<<ir_6_value<<std::endl;
        std::cout << "ir_6 :" << ir_val[5] << std::endl;
        //std::cout<<"ir_7 :"<<ir_6_value<<std::endl;
        std::cout << "ir_7 :" << ir_val[6] << std::endl;


        if (ir_val[0] == 1 && ir_val[1] == 1 && ir_val[2] == 0 && ir_val[3] == 0 && ir_val[4] == 0 && ir_val[5] == 0 && ir_val[6] == 0)
        {
            robot->step(1000);
            leftMotor->setVelocity(0.1);
            rightMotor->setVelocity(TURN_SPEED);

            robot->step(1300);

        }

        else if (ir_val[0] == 0 && ir_val[1] == 0 && ir_val[2] == 0 && ir_val[3] == 0 && ir_val[4] == 0 && ir_val[5] == 1 && ir_val[6] == 1)
        {
            robot->step(1000);
            leftMotor->setVelocity(TURN_SPEED);
            rightMotor->setVelocity(0.1);

            robot->step(1300);

        }





        else
        {
            double error = 0;

            for (int i = 0; i < 7; i++)
            {
                error += ir_val[i] * ir_weight[i];
                //std::cout<<"error "<<error<<std::endl;
            }

            totalError += error;

            //std::cout<<"total error "<<totalError<<std::endl;
            double d = error - previousError;

            //double PID_val = (error*k_p + d*k_d + totalError*k_i);
            double PID_val = (error * k_p + d * k_d + totalError * k_i);
            //std::cout<<"error "<<error<<std::endl; 
            //std::cout<<"PID_val "<<PID_val<<std::endl;


            //leftMotor->setVelocity(-MID_SPEED);
            //right_motor->setVelocity(-MID_SPEED);

            double right_motor_speed = MID_SPEED + PID_val;
            double left_motor_speed = MID_SPEED - PID_val;


            if (right_motor_speed < 0) { right_motor_speed = -0.1; }
            if (right_motor_speed > MAX_SPEED) { right_motor_speed = MAX_SPEED; }

            if (left_motor_speed < 0) { left_motor_speed = -0.1; }
            if (left_motor_speed > MAX_SPEED) { left_motor_speed = MAX_SPEED; }

            //std::cout<<"right_motor"<<right_motor_speed<<std::endl;
            //std::cout<<"left_motor"<<left_motor_speed<<std::endl;

            // Process sensor data here.

            // Enter here functions to send actuator commands, like:
            //  motor->setPosition(10.0);




            leftMotor->setVelocity(-left_motor_speed);
            rightMotor->setVelocity(-right_motor_speed);
            previousError = error;
            std::cout << "right_motor" << right_motor_speed << std::endl;
            std::cout << "left_motor" << left_motor_speed << std::endl;
        }

        std::cout << ir_1_value << ", " << ir_2_value << ", " << ir_3_value << ", " << ir_4_value << ", " << ir_5_value << ", " << ir_6_value << ", " << ir_7_value << ", ";

        if (ir_1_value > 500 &&
            ir_2_value > 500 &&
            ir_3_value > 500 &&
            ir_4_value > 500 &&
            ir_5_value > 500 &&
            ir_6_value > 500 &&
            ir_7_value > 500) {

            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);

            leftMotor->setPosition(0.0);
            rightMotor->setPosition(0.0);

            break;
        }

    };
}

void chamberExplore() {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(-5.0);
    rightMotor->setVelocity(-5.0);

    gripperLift->setVelocity(0.03);
    rightFinger->setVelocity(0.1);
    leftFinger->setVelocity(0.1);

    gripperLift->setPosition(0.13);
    rightFinger->setPosition(0.12);
    leftFinger->setPosition(0.12);

    // initialize sensors
    leftSensor = robot->getDistanceSensor("leftFrontDS");

    leftSensor->enable(TIMESTEP);

    double newFrontDSValue = frontDSLaser->getValue();
    double leftDSValue = leftSensor->getValue();

    bool boxFound = false;

    while (true) {
        robot->step(TIMESTEP);

        newFrontDSValue = frontDSLaser->getValue();
        std::cout << newFrontDSValue << "\n";

        leftDSValue = leftSensor->getValue();
        std::cout << "Left: " << leftDSValue << "\n";

        if (newFrontDSValue < 30) {
            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);
            break;
        }

        if (leftDSValue < 450) {
            advanceTile(0.35);
            turnLeft();

            boxFound = true;

            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);
            break;
        }
    }

    bool pickedUp = false;

    if (boxFound) {
        leftMotor->setVelocity(-5.0);
        rightMotor->setVelocity(-5.0);

        robot->step(TIMESTEP);
        double advancedDistance = frontDSLaser->getValue();

        while (true) {
            robot->step(TIMESTEP);
            std::cout << newFrontDSValue << '\n';

            if (newFrontDSValue < 10) {
                rightFinger->setPosition(0.06);
                leftFinger->setPosition(0.06);
                gripperLift->setPosition(-0.01);

                pickedUp = true;
            }

            if (pickedUp && newFrontDSValue > 10) {
                leftMotor->setVelocity(0.0);
                rightMotor->setVelocity(0.0);

                break;
            }
            newFrontDSValue = frontDSLaser->getValue();
        }

        double finalDistance = frontDSLaser->getValue();

        leftMotor->setVelocity(5.0);
        rightMotor->setVelocity(5.0);

        while (true) {
            robot->step(TIMESTEP);

            newFrontDSValue = frontDSLaser->getValue();

            if (newFrontDSValue > advancedDistance + finalDistance + 40) {
                leftMotor->setVelocity(0.0);
                rightMotor->setVelocity(0.0);

                turnRight();

                break;
            }
        }

        leftMotor->setVelocity(-5.0);
        rightMotor->setVelocity(-5.0);

        while (true) {
            robot->step(TIMESTEP);

            newFrontDSValue = frontDSLaser->getValue();
            std::cout << newFrontDSValue << "\n";

            leftDSValue = leftSensor->getValue();
            std::cout << "Left: " << leftDSValue << "\n";

            if (newFrontDSValue < 45) {
                leftMotor->setVelocity(0.0);
                rightMotor->setVelocity(0.0);
                break;
            }
        }

        turnRight();

        //read values from ir sensors
        double ir_1_value = ir_1->getValue();
        double ir_2_value = ir_2->getValue();
        double ir_3_value = ir_3->getValue();
        double ir_4_value = ir_4->getValue();
        double ir_5_value = ir_5->getValue();
        double ir_6_value = ir_6->getValue();
        double ir_7_value = ir_7->getValue();

        leftMotor->setVelocity(-5.0);
        rightMotor->setVelocity(-5.0);

        while (true) {
            robot->step(TIMESTEP);

            ir_1_value = ir_1->getValue();
            ir_2_value = ir_2->getValue();
            ir_3_value = ir_3->getValue();
            ir_4_value = ir_4->getValue();
            ir_5_value = ir_5->getValue();
            ir_6_value = ir_6->getValue();
            ir_7_value = ir_7->getValue();

            if (ir_1_value < 300 ||
                ir_2_value < 300 ||
                ir_3_value < 300 ||
                ir_4_value < 300 ||
                ir_5_value < 300 ||
                ir_6_value < 300 ||
                ir_7_value < 300) {

                leftMotor->setVelocity(0.0);
                rightMotor->setVelocity(0.0);

                leftMotor->setPosition(0.0);
                rightMotor->setPosition(0.0);

                break;
            }
        }
    }
}