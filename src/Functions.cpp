#include "main.h"



// int limiter(int input){
//     int max = 80;
//     if(input > max){
//         return max;
//     }else if (input < -max){
//         return -max;
//     }else{
//         return input;
//     }
// }
// void drivePID(int distance, int time){
//     left_mg.tare_position();
//     right_mg.tare_position();
//     double error = distance - left_mg.get_position();  
//     double lastError, deriv; 
//     double kp = 0.3;
//     double kd = 3; 
//     double power; 
//     int startTime = pros::millis();
//     int counter = pros::millis()-startTime;
//     while(abs((int)error)> 0.5 && counter <= time){
//         error = distance - left_mg.get_position(); 
//         deriv = error - lastError;
//         lastError = error; 
//         power = limiter(error*kp + deriv*kd); 
//         counter = pros::millis()-startTime;
//         left_mg.move(power);
//         right_mg.move(power);
//         delay(20);
//     }
//     left_mg.move(0);
//     right_mg.move(0);
// }


// // void drivePID(int distance){
// //     left_mg.tare_position();
// //     right_mg.tare_position();
// //     double error = distance - left_mg.get_position();  
// //     double lastError, deriv; 
// //     double kp = 0.3;
// //     double kd = 3; 
// //     double power; 
// //     while(abs(error)> 0.5){
// //         error = distance - left_mg.get_position(); 
// //         deriv = error - lastError;
// //         lastError = error; 
// //         power = error*kp + deriv*kd; 
// //         left_mg.move(power);
// //         right_mg.move(power);
// //         delay(20);
// //     }
// //     left_mg.move(0);
// //     right_mg.move(0);
// // }

// void turnPID(int degrees,int time){
//     imu.tare_rotation();
//     double error = degrees - imu.get_rotation();
//     double kp = 0.5;
//     double power;
//     int startTime = pros::millis();
//     int counter = pros::millis()-startTime;

//     while(abs((int)error)>0.5 && counter <= time){
//         error = degrees - imu.get_rotation();
//         power = error*kp;
//         left_mg.move(-power);
//         right_mg.move(power);
//         delay(20);

//     } 
//     left_mg.move(0);
//     right_mg.move(0);

// }

