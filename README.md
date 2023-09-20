## Robot Challenge README

## Virtual Robot

### Overview

This repository contains code and documentation for this robot designed for a series of challenging tasks. The robot showcases its capabilities in the following tasks:

### Task 1: Line Following

The robot excels at tracking a white line on a black surface, handling both straight and curved paths with precision. IR sensors have been used to track the white line using PID.

### Task 2: Segmented Wall Following

The robot demonstrates its agility by following segmented walls, whether straight, curved, or a combination of both. Ultrasonic sensors have been used to achieve this task.

### Task 3: Dotted Line Following

After mastering wall-following, the robot faces the colored dotted line challenge. It must choose the correct path based on a random color assignment. This path may feature straight and curved dotted lines, leading to the chessboard area.

### Task 4: Chess Board Challenge

In this challenge, the robot becomes a black rook in a chess game. Its mission is to deliver checkmate in just one move and unlock the secret chamber door. The robot starts on the a7 square, parallel to the rows of the chessboard. It must select the black rook on the a7 square and find the checkmate move, using no prior knowledge of the game's position.

Once the robot identifies the checkmating square, it places the rook, opens the chamber door, and collects two boxes from inside. The red carpet area remains off-limits until checkmate is achieved. Afterward, the robot is free to navigate the entire chessboard without colliding with other pieces.

Explore this repository to uncover the robot's journey through these exciting challenges!

## Physical Robot

### Task 1: Line Maze

### Exploration stage

The robot starts at the white starting square and must navigate through the maze, making only 90-degree turns. The exploration stage concludes when the robot reaches the white checkpoint square on the opposite side of the maze. No loops are present within the maze.

### Speeding Stage

After exploring the maze, the robot's next task is to calculate the shortest path using the collected data and make its way back to the starting square from the checkpoint. The robot's performance in this stage will be evaluated based on the speed at which it returns to the starting square.

## Curved Wall

The robot's next challenge is to follow a curved wall to the left without crossing a red line. The wall runs parallel to the maze's entry point and leads to a blind box entrance at its end. The robot must successfully enter the blind box while avoiding penalties for crossing the red line.

## Blind Box

Inside the blind box, the robot will encounter three openings: the entrance, a wrong exit, and the correct exit. To successfully complete this challenge, the robot must exit through the correct exit, identified by a line on the floor.
